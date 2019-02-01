package frc.robot.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.GENERAL;
import frc.robot.Constants.HATCH_ARM;
import frc.robot.Constants.TEST;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.CargoArmState;
import java.util.ArrayList;

public class MkTalon {

  public final TalonSRX masterTalon;
  public final VictorSPX slaveVictor;
  public final TalonSRX slaveTalon;
  private TalonLoc side;
  private ControlMode lastControlMode = null;
  private double lastOutput = Double.NaN;
  private double lastArbFeed = Double.NaN;
  private NeutralMode lastNeutralMode = null;
  private double mMaxRPM = 0.0;
  private final int kLong = GENERAL.kLongCANTimeoutMs;
  private final int kShort = GENERAL.kMediumTimeoutMs;
  private final int kSlot = GENERAL.kPIDLoopIdx;

  /**
   * @param master Talon with Encoder CAN ID
   * @param slave Follower Talon CAN ID
   */
  public MkTalon(int master, int slave, TalonLoc side) {
    masterTalon = new TalonSRX(master);
    if (side == TalonLoc.HatchArm) {
      slaveVictor = null;
      slaveTalon = new TalonSRX(slave);
    } else {
      slaveVictor = new VictorSPX(slave);
      slaveTalon = null;
    }
    this.side = side;
    resetConfig();
  }


  public void resetConfig() {
    CTRE(masterTalon.configFactoryDefault(kLong));
    if (side == TalonLoc.Left_Drive) {
      CTRE(masterTalon.config_kF(kSlot, Constants.DRIVE.kLeftDriveF, kLong));
      masterTalon.setInverted(Constants.DRIVE.kLeftMasterInvert);
      slaveVictor.setInverted(Constants.DRIVE.kLeftSlaveInvert);
      masterTalon.setSensorPhase(Constants.DRIVE.kLeftSensorInvert);
    } else if (side == TalonLoc.Right_Drive) {
      CTRE(masterTalon.config_kF(kSlot, Constants.DRIVE.kRightDriveF, kLong));
      masterTalon.setInverted(Constants.DRIVE.KRightMasterInvert);
      slaveVictor.setInverted(Constants.DRIVE.kRightSlaveInvert);
      masterTalon.setSensorPhase(Constants.DRIVE.kRightSensorInvert);
    } else if (side == TalonLoc.CargoArm) {
      masterTalon.setSensorPhase(CARGO_ARM.ARM_SENSOR_PHASE);
      masterTalon.setInverted(CARGO_ARM.ARM_MASTER_DIRECTION);
      slaveVictor.setInverted(CARGO_ARM.ARM_SLAVE_DIRECTION);
      CTRE(masterTalon.config_kF(kSlot, CARGO_ARM.ARM_F, kLong));
      CTRE(masterTalon.config_kP(kSlot, CARGO_ARM.ARM_P, kLong));
      CTRE(masterTalon.config_kI(kSlot, CARGO_ARM.ARM_I, kLong));
      CTRE(masterTalon.config_kD(kSlot, CARGO_ARM.ARM_D, kLong));
      CTRE(masterTalon.configMotionCruiseVelocity((int) CARGO_ARM.MOTION_MAGIC_CRUISE_VEL, kLong));
      CTRE(masterTalon.configMotionAcceleration((int) CARGO_ARM.MOTION_MAGIC_ACCEL, kLong));
      CTRE(masterTalon.configForwardSoftLimitThreshold((int) CARGO_ARM.ARM_FORWARD_LIMIT));
      CTRE(masterTalon.configReverseSoftLimitThreshold((int) CARGO_ARM.ARM_REVERSE_LIMIT));
    } else if (side == TalonLoc.HatchArm) {
      masterTalon.setSensorPhase(HATCH_ARM.ARM_SENSOR_PHASE);
      masterTalon.setInverted(HATCH_ARM.ARM_MASTER_DIRECTION);
      slaveVictor.setInverted(HATCH_ARM.ARM_SLAVE_DIRECTION);
      CTRE(masterTalon.config_kF(kSlot, HATCH_ARM.ARM_F, kLong));
      CTRE(masterTalon.config_kP(kSlot, HATCH_ARM.ARM_P, kLong));
      CTRE(masterTalon.config_kI(kSlot, HATCH_ARM.ARM_I, kLong));
      CTRE(masterTalon.config_kD(kSlot, HATCH_ARM.ARM_D, kLong));
      CTRE(masterTalon.configMotionCruiseVelocity((int) HATCH_ARM.kMotionMagicCruiseVel, kLong));
      CTRE(masterTalon.configMotionAcceleration((int) HATCH_ARM.kMotionMagicAccel, kLong));
      CTRE(masterTalon.configForwardSoftLimitThreshold((int) HATCH_ARM.ARM_FORWARD_LIMIT));
      CTRE(masterTalon.configReverseSoftLimitThreshold((int) HATCH_ARM.ARM_REVERSE_LIMIT));
    } else if (side == TalonLoc.CargoIntake) {
      masterTalon.setInverted(CARGO_ARM.LEFT_INTAKE_DIRECTION);
      slaveVictor.setInverted(CARGO_ARM.RIGHT_INTAKE_DIRECTION);
      CTRE(slaveTalon.setControlFramePeriod(ControlFrame.Control_3_General, 1000));
      CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, kLong));
      CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, kLong));
      CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, kLong));
      CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, kLong));
      CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, kLong));
      CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kLong));
    }
    if (side == TalonLoc.Left_Drive || side == TalonLoc.Right_Drive) {
      CTRE(masterTalon.config_kP(kSlot, Constants.DRIVE.kDriveP, kLong));
      CTRE(masterTalon.config_kI(kSlot, Constants.DRIVE.kDriveI, kLong));
      CTRE(masterTalon.config_kD(kSlot, Constants.DRIVE.kDriveD, kLong));
      CTRE(masterTalon.configMotionCruiseVelocity((int) Constants.DRIVE.kMotionMagicCruiseNativeVel, kLong));
      CTRE(masterTalon.configMotionAcceleration((int) Constants.DRIVE.kMotionMagicNativeAccel, kLong));
      CTRE(masterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, kLong));
      CTRE(masterTalon.configVelocityMeasurementWindow(2, kLong));
      CTRE(masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kLong));
      CTRE(masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kSlot, kLong));
    } else if (side == TalonLoc.CargoArm || side == TalonLoc.HatchArm) {
      CTRE(masterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, kLong));
      CTRE(masterTalon.configVelocityMeasurementWindow(64, kLong));
      CTRE(masterTalon.configForwardSoftLimitEnable(true, kLong));
      CTRE(masterTalon.configReverseSoftLimitEnable(true, kLong));
      CTRE(masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 10));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 50, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kLong));
      CTRE(masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kSlot, kLong));
      CTRE(masterTalon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, kLong));
      CTRE(masterTalon.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0, 0, kLong));
    }
    masterTalon.selectProfileSlot(kSlot, kSlot);
    masterTalon.setNeutralMode(NeutralMode.Brake);
    CTRE(masterTalon.configNominalOutputForward(0, kLong));
    CTRE(masterTalon.configNominalOutputReverse(0, kLong));
    CTRE(masterTalon.configPeakOutputForward(1.0, kLong));
    CTRE(masterTalon.configPeakOutputReverse(-1.0, kLong));
    CTRE(masterTalon.configVoltageCompSaturation(12.0, kLong));
    CTRE(masterTalon.configVoltageMeasurementFilter(32, kLong));
    masterTalon.enableVoltageCompensation(true);
    CTRE(masterTalon.configNeutralDeadband(0.0, kLong));
    CTRE(masterTalon.configClosedloopRamp(0.0, kLong));
    lastControlMode = ControlMode.PercentOutput;
    lastNeutralMode = NeutralMode.Brake;
    lastOutput = Double.NaN;
    zeroEncoder();

    if (side == TalonLoc.HatchArm) {
      CTRE(slaveTalon.configFactoryDefault(kLong));
      CTRE(masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 1000));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, kLong));
      CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kLong));
    } else {
      CTRE(slaveVictor.configFactoryDefault(kLong));
      slaveVictor.setNeutralMode(NeutralMode.Brake);
      slaveVictor.setControlFramePeriod(ControlFrame.Control_3_General, 5);
      CTRE(slaveVictor.configNominalOutputForward(0, kLong));
      CTRE(slaveVictor.configNominalOutputReverse(0, kLong));
      CTRE(slaveVictor.configPeakOutputForward(1.0, kLong));
      CTRE(slaveVictor.configPeakOutputReverse(-1.0, kLong));
      CTRE(slaveVictor.configVoltageCompSaturation(12.0, kLong));
      slaveVictor.enableVoltageCompensation(true);
      CTRE(slaveVictor.configVoltageMeasurementFilter(32, kLong));
      slaveVictor.follow(masterTalon);
    }
  }

  public boolean isEncoderConnected() {
    return masterTalon.getSensorCollection().getPulseWidthRiseToRiseUs() > 100;
  }

  public synchronized void set(ControlMode mode, double value, NeutralMode nMode) {
    set(mode, value, nMode, 0);
  }

  public synchronized void set(ControlMode mode, double value, NeutralMode nMode, double arbFeed) {
    if (lastNeutralMode != nMode) {
      lastNeutralMode = nMode;
      masterTalon.setNeutralMode(nMode);
      slaveVictor.setNeutralMode(nMode);
    }
    if (mode != lastControlMode || value != lastOutput || arbFeed != lastArbFeed) {
      masterTalon.set(mode, value, DemandType.ArbitraryFeedForward, arbFeed);
      lastOutput = value;
      lastArbFeed = arbFeed;
      lastControlMode = mode;
    }
  }

  public void zeroEncoder() {
    if (side == TalonLoc.Left_Drive || side == TalonLoc.Right_Drive) {
      CTRE(masterTalon.setSelectedSensorPosition(0, kSlot, kShort));
    } else if (side == TalonLoc.CargoArm) {
      CTRE(masterTalon.getSensorCollection()
          .syncQuadratureWithPulseWidth(CARGO_ARM.kBookEnd_0, CARGO_ARM.kBookEnd_1, CARGO_ARM.kCrossOverZero, CARGO_ARM.kOffset, kShort));
    } else if (side == TalonLoc.HatchArm) {
      CTRE(masterTalon.getSensorCollection()
          .syncQuadratureWithPulseWidth(HATCH_ARM.kBookEnd_0, HATCH_ARM.kBookEnd_1, HATCH_ARM.kCrossOverZero, HATCH_ARM.kOffset, kShort));
    }
  }

  public void updateSmartDash(boolean showRPM) {
    if (showRPM) {
      double rp = (((masterTalon.getSelectedSensorVelocity(0)) / 4096.0) * 60 * 10);
      mMaxRPM = mMaxRPM > rp ? mMaxRPM : rp;
      SmartDashboard.putNumber(side.toString() + " RPM", mMaxRPM);
    }
    SmartDashboard.putNumber(side.toString() + " Velocity", getSpeed());
    SmartDashboard.putNumber(side.toString() + " Error", getError());
    SmartDashboard.putNumber(side.toString() + " Master Output", masterTalon.getMotorOutputPercent());
    SmartDashboard.putNumber(side.toString() + " Position", getPosition());
  }

  public synchronized double getSpeed() {
    if (side == TalonLoc.CargoArm || side == TalonLoc.HatchArm) {
      return MkMath.nativeUnitsPer100MstoDegreesPerSec(masterTalon.getSelectedSensorVelocity(kSlot));
    } else {
      return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getSelectedSensorVelocity(kSlot));
    }
  }

  public synchronized double getPosition() {
    if (side == TalonLoc.CargoArm || side == TalonLoc.HatchArm) {
      return MkMath.nativeUnitsToDegrees(masterTalon.getSelectedSensorPosition(kSlot));
    } else {
      return MkMath.nativeUnitsToInches(masterTalon.getSelectedSensorPosition(kSlot));
    }
  }

  private synchronized double getError() {
    if (side == TalonLoc.CargoArm || side == TalonLoc.HatchArm) {
      return MkMath.nativeUnitsToDegrees(lastOutput) - getPosition();
    } else {
      return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getClosedLoopError(kSlot));
    }
  }


  public synchronized double getCurrent() {
    return masterTalon.getOutputCurrent();
  }

  public double getAbsolutePosition() {
    return masterTalon.getSensorCollection().getPulseWidthPosition();
  }

  public int getZer() {
    int pulseWidth = masterTalon.getSensorCollection().getPulseWidthPosition();
    if (pulseWidth > 0) {
      pulseWidth = pulseWidth & 0xFFF;
    } else {
      pulseWidth += (-Math.round(((double) pulseWidth / 4096) - 0.50)) * 4096;
    }
    return pulseWidth + (-CARGO_ARM.kBookEnd_0);
  }

  public boolean checkSystem() {
    boolean check = true;
    ArrayList<Double> currents = new ArrayList<>();
    ArrayList<Double> velocities = new ArrayList<>();
    ArrayList<Double> positions = new ArrayList<>();

    if (side == TalonLoc.Left_Drive || side == TalonLoc.Right_Drive) {
      zeroEncoder();
      masterTalon.set(ControlMode.PercentOutput, 0.0);
      slaveVictor.set(ControlMode.PercentOutput, 1.0);
      Timer.delay(5.0);
      currents.add(getCurrent());
      velocities.add(getSpeed());
      positions.add(getPosition());
      if (getPosition() < TEST.kMinDriveTestPos || getSpeed() < TEST.kMinDriveTestVel
          || masterTalon.getOutputCurrent() > TEST.kMinDriveTestCurrent) {
        Logger.logError("FAILED - " + side.toString() + "Slave FAILED TO REACH REQUIRED SPEED OR POSITION");
        Logger.logMarker(side.toString() + " Slave Test Failed - Vel: " + getSpeed() + " Pos: " + getPosition());
        check = false;
      } else {
        Logger.logMarker(side.toString() + " Slave - Vel: " + getSpeed() + " Pos: " + getPosition());
      }

      zeroEncoder();
      slaveVictor.set(ControlMode.PercentOutput, 0.0);
      masterTalon.set(ControlMode.PercentOutput, 1.0);
      Timer.delay(5.0);
      currents.add(getCurrent());
      velocities.add(getSpeed());
      positions.add(getPosition());
      if (getPosition() < TEST.kMinDriveTestPos || getSpeed() < TEST.kMinDriveTestVel
          || masterTalon.getOutputCurrent() > TEST.kMinDriveTestCurrent) {
        Logger.logError("FAILED - " + side.toString() + "Master FAILED TO REACH REQUIRED SPEED OR POSITION");
        Logger.logMarker(side.toString() + " Master Test Failed - Vel: " + getSpeed() + " Pos: " + getPosition());
        check = false;
      } else {
        Logger.logMarker(side.toString() + " Master - Vel: " + getSpeed() + " Pos: " + getPosition());
      }

      if (currents.size() > 0) {
        Double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();
        if (!Util.allCloseTo(currents, average, TEST.kDriveCurrentEpsilon)) {
          Logger.logError(side.toString() + " Currents varied!!!!!!!!!!!");
          check = true;
        }
      }

      if (positions.size() > 0) {
        Double average = positions.stream().mapToDouble(val -> val).average().getAsDouble();
        if (!Util.allCloseTo(positions, average, TEST.kDrivePosEpsilon)) {
          Logger.logError(side.toString() + " Positions varied!!!!!!!!");
          check = true;
        }
      }

      if (velocities.size() > 0) {
        Double average = velocities.stream().mapToDouble(val -> val).average().getAsDouble();
        if (!Util.allCloseTo(velocities, average, TEST.kDriveVelEpsilon)) {
          Logger.logError(side.toString() + " Velocities varied!!!!!!!!");
          check = true;
        }
      }

    } else if (side == TalonLoc.CargoArm) {
      if (!isEncoderConnected()) {
        Logger.logError("Arm Encoder Not Connected");
        check = false;
      }
      for (CargoArmState state : CargoArmState.values()) {
        if (state != CargoArmState.ENABLE) {
          CargoArm.getInstance().setArmState(state);
          CargoArm.getInstance().setIntakeRollers(-0.25);
          Timer.delay(2);
        }
      }
      resetConfig();
    }

    return check;
  }

  public enum TalonLoc {
    Left_Drive, Right_Drive, HatchArm, CargoArm, CargoIntake
  }

  public void CTRE(ErrorCode errorCode) {
    if (errorCode != ErrorCode.OK) {
      Logger.logError(errorCode.toString());
    }
  }
}
