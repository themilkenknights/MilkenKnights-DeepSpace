package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.drivers.CT;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;

public class CargoArm extends Subsystem {

  private static ArmControlState mArmControlState = ArmControlState.MOTION_MAGIC;
  private static ArmState mArmState = ArmState.ENABLE;
  private final MkTalon armTalon;
  private final MkTalon intakeTalon;
  private boolean armSafety, disCon = false;
  private double startDis, openLoopSetpoint, rollerSetpoint, armPosEnable = 0.0;

  private CargoArm() {
    armTalon = new MkTalon(CAN.kMasterCargoArmTalonID, CAN.kSlaveCargoArmVictorID, TalonLoc.CargoArm);
    intakeTalon = new MkTalon(CAN.kLeftCargoIntakeTalonID, CAN.kRightCargoIntakeVictorID, TalonLoc.CargoIntake);
  }

  public static CargoArm getInstance() {
    return InstanceHolder.mInstance;
  }

  private void armSafetyCheck() {
    if (!armTalon.isEncoderConnected()) {
      if (disCon) {
        if (Timer.getFPGATimestamp() - startDis > 0.25) {
          setArmControlState(ArmControlState.OPEN_LOOP);
          disCon = false;
          startDis = 0;
        }
      } else {
        disCon = true;
        startDis = Timer.getFPGATimestamp();
      }
      Logger.logError("Arm Encoder Not Connected");
    } else {
      if (disCon) {
        disCon = false;
        startDis = 0;
        armTalon.zeroEncoder();
        Timer.delay(0.05);
        setArmControlState(ArmControlState.MOTION_MAGIC);
      }
    }

    if (armTalon.getCurrent() > CARGO_ARM.MAX_SAFE_CURRENT) {
      Logger.logError("Unsafe Current " + armTalon.getCurrent() + " Amps");
      setArmControlState(ArmControlState.OPEN_LOOP);
    }
  }

  private void setEnable() {
    armPosEnable = armTalon.getPosition();
    mArmState = ArmState.ENABLE;
  }

  public void changeSafety() {
    if (armSafety) {
      CT.RE(armTalon.masterTalon.configForwardSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
      CT.RE(armTalon.masterTalon.configReverseSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
      setEnable();
      setArmControlState(ArmControlState.OPEN_LOOP);
      armSafety = false;
    } else {
      armSafety = true;
      CT.RE(armTalon.masterTalon.configForwardSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
      CT.RE(armTalon.masterTalon.configReverseSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
      setArmControlState(ArmControlState.OPEN_LOOP);
    }
  }

  /*
  Step 3: Write setpoints to Talon
   */
  @Override
  public synchronized void writePeriodicOutputs(double timestamp) {
    if (mArmControlState == ArmControlState.OPEN_LOOP) {
      armTalon.set(ControlMode.PercentOutput, openLoopSetpoint, NeutralMode.Brake);
    } else if (mArmControlState == ArmControlState.MOTION_MAGIC) {
      double armFeed = MkMath.sin(armTalon.getPosition() + CARGO_ARM.ARM_OFFSET) * CARGO_ARM.FEED_CONSTANT;
      if (mArmState.equals(ArmState.ENABLE)) {
        armTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(armPosEnable), NeutralMode.Brake);
      } else {
        armTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(mArmState.state), NeutralMode.Brake, -armFeed);
      }
    } else {
      Logger.logError("Unexpected arm control state: " + mArmControlState);
    }
  }

  @Override
  public void outputTelemetry() {
    armTalon.updateSmartDash(false);
    SmartDashboard.putString("Arm Desired Position", mArmState.toString());
    SmartDashboard.putString("Arm Control Mode", mArmControlState.toString());
    SmartDashboard.putBoolean("Arm Status", armTalon.isEncoderConnected());
  }

  @Override
  public void zero(double timestamp) {
    synchronized (CargoArm.this) {
      armTalon.zeroEncoder();
      setEnable();
      if (armTalon.getZer() > GENERAL.kTicksPerRev) {
        Logger.logMarker("Arm Absolution Position > 4096");
        setArmControlState(ArmControlState.OPEN_LOOP);
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
    synchronized (CargoArm.this) {
      armSafetyCheck();
      updateRollers();
    }
  }

  @Override
  public void onStop(double timestamp) {
    setIntakeRollers(0);
  }

  @Override
  public boolean checkSystem() {
    return armTalon.checkSystem();
  }

  private void updateRollers() {
    intakeTalon.set(ControlMode.PercentOutput, rollerSetpoint, NeutralMode.Brake);
  }

  public synchronized void setOpenLoop(double output) {
    if (armSafety) {
      setArmControlState(ArmControlState.OPEN_LOOP);
    } else {
      Logger.logError("Failed to set Arm Open Loop Ouput: Arm Safety Not Enabled");
    }
  }

  public synchronized void setArmState(ArmState state) {
    if (!armSafety) {
      setArmControlState(ArmControlState.MOTION_MAGIC);
      mArmState = state;
    } else {
      Logger.logError("Failed to set Arm State: Arm Safety Enabled");
    }
  }

  public void setIntakeRollers(double output) {
    rollerSetpoint = output;
  }

  public ArmControlState getArmControlState() {
    return mArmControlState;
  }

  public ArmState getArmState() {
    return mArmState;
  }

  public enum ArmControlState {
    MOTION_MAGIC, // Closed Loop Motion Profile following on the talons used in nearly all circumstances
    OPEN_LOOP // Direct PercentVBus control of the arm as a failsafe
  }

  private void setArmControlState(ArmControlState state) {
    if (state == ArmControlState.MOTION_MAGIC && mArmControlState != ArmControlState.MOTION_MAGIC) {
      setEnable();
    } else if (state == ArmControlState.OPEN_LOOP && mArmControlState != ArmControlState.OPEN_LOOP) {
      openLoopSetpoint = 0.0;
    }
    mArmControlState = state;
  }

  public enum ArmState {
    ENABLE(0), //State directly after robot is enabled (not mapped to a specific angle)
    OPPOSITE_SWITCH_PLACE(78.5), //Outtakes into the switch on the backside of the robot
    SECOND_SWITCH_PLACE(219.5);

    public final double state;

    ArmState(final double state) {
      this.state = state;
    }
  }

  private static class InstanceHolder {

    private static final CargoArm mInstance = new CargoArm();
  }
}
