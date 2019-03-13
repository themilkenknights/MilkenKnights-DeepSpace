package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.lib.drivers.CT;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;

public class CargoArm extends Subsystem {

  private static CargoArmControlState mCargoArmControlState = CargoArmControlState.MOTION_MAGIC;
  private static CargoArmState mCargoArmState = CargoArmState.ENABLE;
  public final MkTalon mIntakeTalon;
  private final MkTalon mArmTalon;
  private boolean mDisCon = false;
  private boolean mSoftLimitState = true;
  private double mStartDis, mOpenLoopSetpoint, mRollerSetpoint, mArmPosEnable = 0.0;
  private NetworkTableEntry mAbsPos, mDesiredState, mControlMode, mStatus;

  private CargoArm() {
    ShuffleboardTab mCargoArmTab = Shuffleboard.getTab("Cargo Arm");
    ShuffleboardTab mIntakeRollersTab = Shuffleboard.getTab("General");

    mAbsPos = mCargoArmTab.add("Absolute Pos", 0.0).getEntry();
    mControlMode = mCargoArmTab.add("Control Mode", "").getEntry();
    mStatus = mCargoArmTab.add("Status", false).getEntry();
    mDesiredState = mCargoArmTab.add("Desired State", "").getEntry();

    mArmTalon = new MkTalon(CAN.kMasterCargoArmTalonID, CAN.kSlaveCargoArmVictorID, TalonLoc.Cargo_Arm, mCargoArmTab);
    mIntakeTalon = new MkTalon(CAN.kLeftCargoIntakeTalonID, CAN.kRightCargoIntakeTalonID, TalonLoc.Cargo_Intake,
        mIntakeRollersTab);
  }

  public static CargoArm getInstance() {
    return InstanceHolder.mInstance;
  }

  public PigeonIMU getPigeon() {
    return mIntakeTalon.mPigeon;
  }

  public synchronized void onQuickLoop(double timestamp) {
    if (mCargoArmControlState == CargoArmControlState.OPEN_LOOP) {
      mArmTalon.set(ControlMode.PercentOutput, mOpenLoopSetpoint, NeutralMode.Brake);
    } else if (mCargoArmControlState == CargoArmControlState.MOTION_MAGIC) {
      if (mCargoArmState.equals(CargoArmState.ENABLE)) {
        mArmTalon.set(ControlMode.MotionMagic, MkMath.degreesToNativeUnits(mArmPosEnable), NeutralMode.Brake);
      } else {
        double armFeed = MkMath.sin(mArmTalon.getPosition() + CARGO_ARM.kArmOffset) * CARGO_ARM.kFeedConstant;
        mArmTalon.set(ControlMode.MotionMagic, MkMath.degreesToNativeUnits(mCargoArmState.state),
            DemandType.ArbitraryFeedForward, -armFeed, NeutralMode.Brake);
      }
    } else {
      Logger.logErrorWithTrace("Unexpected Cargo Arm Control State: " + mCargoArmControlState);
    }
    mIntakeTalon.set(ControlMode.PercentOutput, mRollerSetpoint, NeutralMode.Brake);
  }

  /**
   * The arm encoder was getting periodically briefly disconnected during matches so this method waits 250ms before
   * switching to open loop control.
   *
   * <p>This method also checks for unsafe current output and will automatically switches to open
   * loop control.
   */
  @Override
  public synchronized void safetyCheck(double timestamp) {
    mArmTalon.checkForReset();
    if (!mArmTalon.isEncoderConnected()) {
      if (mDisCon) {
        setOpenLoop(0.0);
        mDisCon = false;
        Logger.logError("Cargo Arm Encoder Disconnected");
      } else {
        mDisCon = true;
      }
    }

    if (mArmTalon.getCurrent() > CARGO_ARM.kMaxSafeCurrent) {
      setOpenLoop(0.0);
      Logger.logError("Unsafe Current on Cargo Arm " + mArmTalon.getCurrent() + " Amps");
    }
  }

  @Override
  public void outputTelemetry(double timestamp) {
    mArmTalon.updateShuffleboard();
    mDesiredState.setString(mCargoArmState.toString());
    mControlMode.setString(mCargoArmControlState.toString());
    /*mAbsPos.setDouble(mArmTalon.masterTalon.getSensorCollection().getPulseWidthPosition());
    mStatus.setBoolean(mArmTalon.isEncoderConnected());*/
  }

  @Override
  public void teleopInit(double timestamp) {
    zeroEncoder();
  }

  @Override
  public void autonomousInit(double timestamp) {
    zeroEncoder();
  }

  @Override
  public void onStop(double timestamp) {
    setIntakeRollers(0);
  }

  @Override
  public void onRestart(double timestamp) {
    mArmTalon.checkForErrorInit();
    mIntakeTalon.checkForErrorInit();
  }

  @Override
  public boolean checkSystem() {
    return mArmTalon.checkSystem() && mIntakeTalon.checkSystem();
  }

  public synchronized void setIntakeRollers(double output) {
    mRollerSetpoint = output;
  }

  public void zeroEncoder() {
    mArmTalon.zeroEncoder();
    setEnable();
  }

  public synchronized void setOpenLoop(double output) {
    if (mCargoArmControlState != CargoArmControlState.OPEN_LOOP) {
      setArmControlState(CargoArmControlState.OPEN_LOOP);
    }
    mOpenLoopSetpoint = output;
  }

  private synchronized void setArmControlState(CargoArmControlState state) {
    if (state == CargoArmControlState.MOTION_MAGIC && mCargoArmControlState != CargoArmControlState.MOTION_MAGIC) {
      setEnable();
      Logger.logMarker("Switching to Cargo Arm Motion Magic");
    } else if (state == CargoArmControlState.OPEN_LOOP && mCargoArmControlState != CargoArmControlState.OPEN_LOOP) {
      mOpenLoopSetpoint = 0.0;
      Logger.logMarker("Switching to Cargo Arm Open Loop");
    }
    mCargoArmControlState = state;
  }

  private void setEnable() {
    mArmPosEnable = mArmTalon.getPosition();
    mCargoArmState = CargoArmState.ENABLE;
  }

  public boolean isSpearLimitTriggered() {
    return mIntakeTalon.slaveTalon.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public CargoArmState getArmState() {
    return mCargoArmState;
  }

  public synchronized void setArmState(CargoArmState state) {
    if (mCargoArmControlState != CargoArmControlState.MOTION_MAGIC) {
      setArmControlState(CargoArmControlState.MOTION_MAGIC);
    }
    mCargoArmState = state;
  }

  public void disableSoftLimit() {
    CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(false, 0));
    CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(false, 0));
  }

  public enum CargoArmControlState {
    MOTION_MAGIC, // Closed Loop Motion Profile following on the talons used in nearly all
    // circumstances
    OPEN_LOOP // Direct PercentVBus control of the arm as a failsafe
  }

  public enum CargoArmState {
    ENABLE(0.0), // State directly after robot is enabled (not mapped to a specific angle)
    INTAKE(182.0),
    FORWARD_ROCKET_LEVEL_ONE(120.0),
    FORWARD_ROCKET_LEVEL_TWO(75.0),
    REVERSE_CARGOSHIP(21.0);

    public final double state;

    CargoArmState(final double state) {
      this.state = state;
    }
  }

  private static class InstanceHolder {

    private static final CargoArm mInstance = new CargoArm();
  }
}
