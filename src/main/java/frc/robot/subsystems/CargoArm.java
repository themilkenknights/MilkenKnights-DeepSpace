package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ARM;
import frc.robot.Constants.CAN;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLocation;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;

public class CargoArm extends Subsystem {

  public static ArmControlState mArmControlState = ArmControlState.MOTION_MAGIC;
  public static ArmState mArmState = ArmState.ENABLE;
  private final MkTalon armTalon;
  private final VictorSPX leftIntakeRollerTalon;
  private final VictorSPX rightIntakeRollerTalon;
  public PeriodicIO mPeriodicIO;
  private boolean armSafety = true;
  private double armPosEnable = 0;
  private double rollerSetpoint = 0;
  private double startDis = 0;
  private boolean disCon = false;

  private CargoArm() {
    mPeriodicIO = new PeriodicIO();
    armTalon = new MkTalon(CAN.kMasterCargoArmMotorID, CAN.kSlaveCargoArmMotorID, TalonLocation.CargoIntake);
    armTalon.masterTalon.setSensorPhase(ARM.ARM_SENSOR_PHASE);
    armTalon.masterTalon.configForwardSoftLimitThreshold((int) ARM.ARM_FORWARD_LIMIT);
    armTalon.masterTalon.configReverseSoftLimitThreshold((int) ARM.ARM_REVERSE_LIMIT);
    armTalon.masterTalon.configForwardSoftLimitEnable(true);
    armTalon.masterTalon.configReverseSoftLimitEnable(true);
    leftIntakeRollerTalon = new VictorSPX(CAN.kLeftCargoIntakeMotorID);
    rightIntakeRollerTalon = new VictorSPX(CAN.kRightCargoIntakeMotorID);
    leftIntakeRollerTalon.setNeutralMode(NeutralMode.Brake);
    rightIntakeRollerTalon.setNeutralMode(NeutralMode.Brake);
    armTalon.masterTalon.setInverted(ARM.ARM_MASTER_DIRECTION);
    armTalon.slaveTalon.setInverted(ARM.ARM_SLAVE_DIRECTION);
    leftIntakeRollerTalon.setInverted(ARM.LEFT_INTAKE_DIRECTION);
    rightIntakeRollerTalon.setInverted(ARM.RIGHT_INTAKE_DIRECTION);
  }

  public static CargoArm getInstance() {
    return InstanceHolder.mInstance;
  }

  private void armSafetyCheck() {
    if (!armTalon.isEncoderConnected()) {
      if (disCon) {
        if (Timer.getFPGATimestamp() - startDis > 0.5) {
          mArmControlState = ArmControlState.OPEN_LOOP;
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
        armTalon.zeroAbsolute();
        Timer.delay(0.05);
        setEnable();
        mArmControlState = ArmControlState.MOTION_MAGIC;
      }
    }

    if (armTalon.getCurrent() > ARM.MAX_SAFE_CURRENT && armSafety) {
      Logger.logError("Unsafe Current " + armTalon.getCurrent() + " Amps");
      mArmControlState = ArmControlState.OPEN_LOOP;
    }
  }

  public void setEnable() {
    armPosEnable = armTalon.getPosition();
    mArmState = ArmState.ENABLE;
  }

  public void changeSafety() {
    armSafety = !armSafety;
    armTalon.masterTalon.configForwardSoftLimitEnable(armSafety);
    armTalon.masterTalon.configReverseSoftLimitEnable(armSafety);
  }

  /*
 Step 1: Read inputs from CAN
  */
  @Override
  public synchronized void readPeriodicInputs(double timestamp) {
    mPeriodicIO.controlMode = mArmControlState.toString();
    mPeriodicIO.output = armTalon.masterTalon.getMotorOutputPercent();
    mPeriodicIO.position = armTalon.getPosition();
    mPeriodicIO.velocity = armTalon.getSpeed();
    mPeriodicIO.setpoint = mArmState.state;
    mPeriodicIO.timestamp = timestamp;
    mPeriodicIO.current = armTalon.getCurrent();
  }

  /*
  Step 3: Write setpoints to Talon
   */
  @Override
  public synchronized void writePeriodicOutputs(double timestamp) {
    if (mArmControlState == ArmControlState.OPEN_LOOP) {
      armTalon.set(ControlMode.PercentOutput, mPeriodicIO.setpoint, NeutralMode.Brake);
    } else if (mArmControlState == ArmControlState.MOTION_MAGIC) {
      double armFeed = MkMath.sin(armTalon.getPosition() + ARM.ARM_OFFSET) * ARM.FEED_CONSTANT;
      if (mArmState.equals(ArmState.ENABLE)) {
        armTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(armPosEnable), NeutralMode.Brake);
      } else {
        armTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(mPeriodicIO.setpoint), NeutralMode.Brake, -armFeed);
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
  public void onStart(double timestamp) {
    synchronized (CargoArm.this) {
      armTalon.zeroAbsolute();
      if (armTalon.getZer() > GENERAL.kTicksPerRev) {
        Logger.logMarker("Arm Absolution Position > 4096");
        mArmControlState = ArmControlState.OPEN_LOOP;
      }
      armPosEnable = armTalon.getPosition();
      mArmState = ArmState.ENABLE;
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
      switch (mArmControlState) {
        case MOTION_MAGIC:
          return;
        case OPEN_LOOP:
          return;
        default:
          Logger.logError("Unexpected arm control state: " + mArmControlState);
          break;
      }
    }
  }

  @Override
  public void onStop(double timestamp) {
    setIntakeRollers(0);
  }

  @Override
  public boolean checkSystem() {
    boolean check = true;
    if (!armTalon.isEncoderConnected()) {
      Logger.logError("Arm Encoder Not Connected");
      check = false;
    }
    if (mArmControlState == ArmControlState.MOTION_MAGIC) {
      for (ArmState state : ArmState.values()) {
        if (state != ArmState.ENABLE) {
          mArmState = state;
          setIntakeRollers(-0.25);
          Timer.delay(2);
        }
      }
      armTalon.resetMasterConfig();
      armTalon.resetSlaveConfig();
    } else {
      Logger.logError("Arm Test Failed");
      check = false;
    }

    return check;
  }

  public void zeroRel() {
    armTalon.resetEncoder();
    armPosEnable = armTalon.getPosition();
  }

  public void updateRollers() {
    leftIntakeRollerTalon.set(ControlMode.PercentOutput, rollerSetpoint);
    rightIntakeRollerTalon.set(ControlMode.PercentOutput, rollerSetpoint);
  }

  public void setOpenLoop(double output) {
    armTalon.set(ControlMode.PercentOutput, output, NeutralMode.Brake);
  }

  public void setIntakeRollers(double output) {
    rollerSetpoint = output;
  }

  public enum ArmControlState {
    MOTION_MAGIC, // Closed Loop Motion Profile following on the talons used in nearly all circumstances
    OPEN_LOOP // Direct PercentVBus control of the arm as a failsafe
  }

  public enum ArmState {
    ENABLE(0), //State directly after robot is enabled (not mapped to a specific angle)
    OPPOSITE_STOW(28.5), //Used to Outtake into the exchange or store cube at start of auto
    OPPOSITE_SWITCH_PLACE(78.5), //Outtakes into the switch on the backside of the robot
    SWITCH_PLACE(163.5), //Main switch outtake position
    SECOND_SWITCH_PLACE(219.5), //Used to intake cubes from the second-floor of the pyramid
    INTAKE(243.5); //Intake Setpoint to get cubes from the ground

    public final double state;

    ArmState(final double state) {
      this.state = state;
    }
  }

  public static class PeriodicIO {

    public double timestamp;
    public String controlMode;
    public double output;
    public double position;
    public double velocity;
    public double setpoint;
    public double current;
  }

  private static class InstanceHolder {

    private static final CargoArm mInstance = new CargoArm();
  }
}
