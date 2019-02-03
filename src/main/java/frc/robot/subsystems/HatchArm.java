package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.GENERAL;
import frc.robot.Constants.HATCH_ARM;
import frc.robot.lib.drivers.CT;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;

public class HatchArm extends Subsystem {

  private static HatchIntakeControlState mHatchIntakeControlState = HatchIntakeControlState.MOTION_MAGIC;
  private static HatchIntakeState mHatchIntakeState = HatchIntakeState.ENABLE;
  private static HatchMechanismState mHatchMechanismState = HatchMechanismState.STOWED;

  private final MkTalon mArmTalon;
  private HatchArmState mHatchArmState;
  private Solenoid mArmSolenoid;
  private boolean mArmSafety, mDisCon = false;
  private double mStartDis, mOpenLoopSetpoint, mArmPosEnable, mTransferTime = 0.0;

  private HatchArm() {
    mArmSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, HATCH_ARM.kHatchArmChannel);
    mHatchArmState = HatchArmState.STOW;
    mArmTalon = new MkTalon(CAN.kGroundHatchArmTalonID, CAN.kHatchLimitSwitchTalonID, TalonLoc.HatchArm);
  }

  public static HatchArm getInstance() {
    return HatchArm.InstanceHolder.mInstance;
  }

  private void armSafetyCheck() {
    if (!mArmTalon.isEncoderConnected()) {
      if (mDisCon) {
        if (Timer.getFPGATimestamp() - mStartDis > 0.25) {
          setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
          mDisCon = false;
          mStartDis = 0;
        }
      } else {
        mDisCon = true;
        mStartDis = Timer.getFPGATimestamp();
      }
      Logger.logCriticalError("Arm Encoder Not Connected");
    } else {
      if (mDisCon) {
        mDisCon = false;
        mStartDis = 0;
        mArmTalon.zeroEncoder();
        Timer.delay(0.05);
        setHatchIntakeControlState(HatchIntakeControlState.MOTION_MAGIC);
      }
    }

    if (mArmTalon.getCurrent() > CARGO_ARM.MAX_SAFE_CURRENT) {
      Logger.logCriticalError("Unsafe Current " + mArmTalon.getCurrent() + " Amps");
      setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
    }
  }

  private void setEnable() {
    mArmPosEnable = mArmTalon.getPosition();
    mHatchIntakeState = HatchIntakeState.ENABLE;
  }

  public boolean hatchOnArmLimit() {
    return mArmTalon.slaveTalon.getSensorCollection().isRevLimitSwitchClosed();
  }

  public void changeSafety() {
    if (mArmSafety) {
      CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
      CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
      setEnable();
      setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
      mArmSafety = false;
    } else {
      mArmSafety = true;
      CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
      CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
      setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
    }
  }

  /*
  Step 3: Write setpoints to Talon
   */
  @Override
  public synchronized void writePeriodicOutputs(double timestamp) {
    if (mHatchIntakeControlState == HatchIntakeControlState.OPEN_LOOP) {
      mArmTalon.set(ControlMode.PercentOutput, mOpenLoopSetpoint, NeutralMode.Brake);
    } else if (mHatchIntakeControlState == HatchIntakeControlState.MOTION_MAGIC) {
      if (mHatchIntakeState == HatchIntakeState.ENABLE) {
        mArmTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(mArmPosEnable), NeutralMode.Brake);
      } else {
        mArmTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(mHatchIntakeState.state), NeutralMode.Brake, 0.0);
      }
    } else {
      Logger.logCriticalError("Unexpected arm control state: " + mHatchIntakeControlState);
    }
  }

  @Override
  public void outputTelemetry() {
    mArmTalon.updateSmartDash(false);
    SmartDashboard.putString("Arm Desired Position", mHatchIntakeState.toString());
    SmartDashboard.putString("Arm Control Mode", mHatchIntakeControlState.toString());
    SmartDashboard.putBoolean("Arm Status", mArmTalon.isEncoderConnected());
  }

  @Override
  public void zero(double timestamp) {
    synchronized (HatchArm.this) {
      mArmTalon.zeroEncoder();
      setEnable();
      if (mArmTalon.getZer() > GENERAL.kTicksPerRev) {
        Logger.logMarker("Arm Absolution Position > 4096");
        setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
      }
    }
  }

  /**
   * Updated from mEnabledLoop in Robot.java
   *
   * @param timestamp Time in seconds since code start
   */
  @Override
  public void onLoop(double timestamp) {
    synchronized (HatchArm.this) {
      armSafetyCheck();
      switch (mHatchMechanismState) {
        case STOWED:
          break;
        case GROUND_INTAKE:
          break;
        case STATION_INTAKE:
          if (mArmTalon.slaveTalon.getSensorCollection().isRevLimitSwitchClosed()) {
            setHatchMechanismState(HatchMechanismState.STOWED);
          }
          break;
        case TRANSFER:
          if (mArmTalon.slaveTalon.getSensorCollection().isRevLimitSwitchClosed()) {
            setHatchArmPosition(HatchArmState.STOW);
            mTransferTime = Timer.getFPGATimestamp();
          }
          if (Timer.getFPGATimestamp() - mTransferTime > 0.1) {
            setHatchMechanismState(HatchMechanismState.STOWED);
            mTransferTime = 0.0;
          }
          break;
        default:
          Logger.logCriticalError("Unexpected drive control state: " + mHatchMechanismState);
          break;
      }

    }
  }

  @Override
  public void onStop(double timestamp) {
    setHatchArmPosition(HatchArmState.STOW);
  }

  @Override
  public boolean checkSystem() {
    setHatchArmPosition(HatchArmState.PLACE);
    Logger.logMarker("Set to Place");
    Timer.delay(1.0);
    setHatchArmPosition(HatchArmState.STOW);
    Timer.delay(1.0);
    Logger.logMarker("Set to Stow");
    return mArmTalon.checkSystem();
  }

  public synchronized void setIntakeOpenLoop(double output) {
    if (mArmSafety) {
      setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
    } else {
      Logger.logCriticalError("Failed to set Arm Open Loop Ouput: Arm Safety Not Enabled");
    }
  }

  private void setHatchIntakeControlState(HatchIntakeControlState state) {
    if (state == HatchIntakeControlState.MOTION_MAGIC && mHatchIntakeControlState != HatchIntakeControlState.MOTION_MAGIC) {
      setEnable();
    } else if (state == HatchIntakeControlState.OPEN_LOOP && mHatchIntakeControlState != HatchIntakeControlState.OPEN_LOOP) {
      mOpenLoopSetpoint = 0.0;
    }
    mHatchIntakeControlState = state;
  }

  private synchronized void setHatchIntakePosition(HatchIntakeState state) {
    if (!mArmSafety) {
      setHatchIntakeControlState(HatchIntakeControlState.MOTION_MAGIC);
      mHatchIntakeState = state;
    } else {
      Logger.logCriticalError("Failed to set Arm State: Arm Safety Enabled");
    }
  }

  public void setHatchMechanismState(HatchMechanismState state) {
    switch (state) {
      case TRANSFER:
        setHatchIntakePosition(HatchIntakeState.TRANSFER_POINT);
      case GROUND_INTAKE:
        setHatchIntakePosition(HatchIntakeState.INTAKE_POINT);
        setHatchArmPosition(HatchArmState.STOW);
      case STOWED:
        setHatchArmPosition(HatchArmState.STOW);
        setHatchIntakePosition(HatchIntakeState.STOW_POINT);
      case STATION_INTAKE:
        setHatchArmPosition(HatchArmState.PLACE);
        setHatchIntakePosition(HatchIntakeState.STOW_POINT);
      case PLACING:
        setHatchArmPosition(HatchArmState.PLACE);
        setHatchIntakePosition(HatchIntakeState.STOW_POINT);
      default:
        Logger.logCriticalError("Unexpected Hatch Mechanism: " + mHatchMechanismState);
        break;
    }
    mHatchMechanismState = state;
  }

  /*
  Move Hatch Arm to Stow or Place
   */
  private synchronized void setHatchArmPosition(HatchArmState armState) {
    mArmSolenoid.set(armState.state);
    Logger.logMarker("Set Hatch Arm to " + armState.toString());
  }


  public enum HatchIntakeControlState {
    MOTION_MAGIC, // Closed Loop Motion Profile following on the talons used in nearly all circumstances
    OPEN_LOOP // Direct PercentVBus control of the arm as a failsafe
  }

  public enum HatchIntakeState {
    ENABLE(0), //State directly after robot is enabled (not mapped to a specific angle)
    INTAKE_POINT(200.0),
    TRANSFER_POINT(78.5), //Outtakes into the switch on the backside of the robot
    STOW_POINT(219.5);

    public final double state;

    HatchIntakeState(final double state) {
      this.state = state;
    }
  }

  public enum HatchArmState {
    PLACE(HATCH_ARM.kHatchArmPlaceState), STOW(!HATCH_ARM.kHatchArmPlaceState);
    public final boolean state;

    HatchArmState(final boolean state) {
      this.state = state;
    }
  }

  public enum HatchMechanismState {
    GROUND_INTAKE, TRANSFER, STATION_INTAKE, STOWED, PLACING
  }


  private static class InstanceHolder {

    private static final HatchArm mInstance = new HatchArm();
  }
}
