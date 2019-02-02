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

  private static CargoArmControlState mCargoArmControlState = CargoArmControlState.MOTION_MAGIC;
  private static CargoArmState mCargoArmState = CargoArmState.ENABLE;
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
          setArmControlState(CargoArmControlState.OPEN_LOOP);
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
        setArmControlState(CargoArmControlState.MOTION_MAGIC);
      }
    }

    if (armTalon.getCurrent() > CARGO_ARM.MAX_SAFE_CURRENT) {
      Logger.logError("Unsafe Current " + armTalon.getCurrent() + " Amps");
      setArmControlState(CargoArmControlState.OPEN_LOOP);
    }
  }

  private void setEnable() {
    armPosEnable = armTalon.getPosition();
    mCargoArmState = CargoArmState.ENABLE;
  }

  public void changeSafety() {
    if (armSafety) {
      CT.RE(armTalon.masterTalon.configForwardSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
      CT.RE(armTalon.masterTalon.configReverseSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
      setEnable();
      setArmControlState(CargoArmControlState.OPEN_LOOP);
      armSafety = false;
    } else {
      armSafety = true;
      CT.RE(armTalon.masterTalon.configForwardSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
      CT.RE(armTalon.masterTalon.configReverseSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
      setArmControlState(CargoArmControlState.OPEN_LOOP);
    }
  }

  /*
  Step 3: Write setpoints to Talon
   */
  @Override
  public synchronized void writePeriodicOutputs(double timestamp) {
    if (mCargoArmControlState == CargoArmControlState.OPEN_LOOP) {
      armTalon.set(ControlMode.PercentOutput, openLoopSetpoint, NeutralMode.Brake);
    } else if (mCargoArmControlState == CargoArmControlState.MOTION_MAGIC) {
      double armFeed = MkMath.sin(armTalon.getPosition() + CARGO_ARM.kArmOffset) * CARGO_ARM.kFeedConstant;
      if (mCargoArmState.equals(CargoArmState.ENABLE)) {
        armTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(armPosEnable), NeutralMode.Brake);
      } else {
        armTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(mCargoArmState.state), NeutralMode.Brake, -armFeed);
      }
    } else {
      Logger.logError("Unexpected arm control state: " + mCargoArmControlState);
    }
  }

  @Override
  public void outputTelemetry() {
    armTalon.updateSmartDash(false);
    SmartDashboard.putString("Arm Desired Position", mCargoArmState.toString());
    SmartDashboard.putString("Arm Control Mode", mCargoArmControlState.toString());
    SmartDashboard.putBoolean("Arm Status", armTalon.isEncoderConnected());
  }

  @Override
  public void zero(double timestamp) {
    synchronized (CargoArm.this) {
      armTalon.zeroEncoder();
      setEnable();
      if (armTalon.getZer() > GENERAL.kTicksPerRev) {
        Logger.logMarker("Arm Absolution Position > 4096");
        setArmControlState(CargoArmControlState.OPEN_LOOP);
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
      setArmControlState(CargoArmControlState.OPEN_LOOP);
    } else {
      Logger.logError("Failed to set Arm Open Loop Ouput: Arm Safety Not Enabled");
    }
  }

  public synchronized void setArmState(CargoArmState state) {
    if (!armSafety) {
      setArmControlState(CargoArmControlState.MOTION_MAGIC);
      mCargoArmState = state;
    } else {
      Logger.logError("Failed to set Arm State: Arm Safety Enabled");
    }
  }

  public void setIntakeRollers(double output) {
    rollerSetpoint = output;
  }

  public CargoArmControlState getArmControlState() {
    return mCargoArmControlState;
  }

  public CargoArmState getArmState() {
    return mCargoArmState;
  }

  public enum CargoArmControlState {
    MOTION_MAGIC, // Closed Loop Motion Profile following on the talons used in nearly all circumstances
    OPEN_LOOP // Direct PercentVBus control of the arm as a failsafe
  }

  private void setArmControlState(CargoArmControlState state) {
    if (state == CargoArmControlState.MOTION_MAGIC && mCargoArmControlState != CargoArmControlState.MOTION_MAGIC) {
      setEnable();
    } else if (state == CargoArmControlState.OPEN_LOOP && mCargoArmControlState != CargoArmControlState.OPEN_LOOP) {
      openLoopSetpoint = 0.0;
    }
    mCargoArmControlState = state;
  }

  public enum CargoArmState {
    ENABLE(0.0), //State directly after robot is enabled (not mapped to a specific angle)
    INTAKE(150.0),
    STOW(80.0),
    PLACE(100.0), //Outtakes into the switch on the backside of the robot
    PLACE_REVERSE(30.0);

    public final double state;

    CargoArmState(final double state) {
      this.state = state;
    }
  }

  private static class InstanceHolder {

    private static final CargoArm mInstance = new CargoArm();
  }
}
