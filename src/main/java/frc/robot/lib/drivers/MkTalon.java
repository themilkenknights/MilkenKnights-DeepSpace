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
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.GENERAL;
import frc.robot.Constants.TEST;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.ArmState;
import java.util.ArrayList;

public class MkTalon {

  public final TalonSRX masterTalon;
  public final VictorSPX slaveTalon;
  private TalonLocation side;
  private ControlMode lastControlMode = null;
  private double lastOutput = Double.NaN;
  private NeutralMode lastNeutralMode = null;
  private double maxRPM = 0;

  /**
   * @param master Talon with Encoder CAN ID
   * @param slave Follower Talon CAN ID
   */
  public MkTalon(int master, int slave, TalonLocation side) {
    masterTalon = new TalonSRX(master);
    slaveTalon = new VictorSPX(slave);
    this.side = side;
    resetMasterConfig();
    resetSlaveConfig();
  }

  public MkTalon(int master, TalonLocation loc) {
    masterTalon = new TalonSRX(master);
    slaveTalon = null;
    this.side = loc;
    resetMasterConfig();
  }

  private synchronized double getError() {
    if (side == TalonLocation.CargoIntake) {
      return MkMath.nativeUnitsToDegrees(masterTalon.getClosedLoopError(Constants.GENERAL.kPIDLoopIdx));
    } else {
      return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getClosedLoopError(Constants.GENERAL.kPIDLoopIdx));
    }
  }

  public void resetMasterConfig() {
    CTRE(masterTalon.configFactoryDefault(GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon.configAllSettings(new TalonSRXConfiguration(), GENERAL.kLongCANTimeoutMs));
    if (side.equals(TalonLocation.Left_Drive)) {
      CTRE(masterTalon.config_kF(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kLeftDriveF, GENERAL.kLongCANTimeoutMs));
    } else if (side.equals(TalonLocation.Right_Drive)) {
      CTRE(masterTalon.config_kF(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kRightDriveF, GENERAL.kLongCANTimeoutMs));
    }
    if (side == TalonLocation.Left_Drive || side == TalonLocation.Right_Drive) {
      CTRE(masterTalon.config_kP(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kDriveP, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.config_kI(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kDriveI, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.config_kD(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kDriveD, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.configMotionCruiseVelocity((int) Constants.DRIVE.kMotionMagicCruiseNativeVel, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.configMotionAcceleration((int) Constants.DRIVE.kMotionMagicNativeAccel, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.configVelocityMeasurementWindow(32));
    } else if (side == TalonLocation.CargoIntake) {
      CTRE(masterTalon.config_kF(Constants.GENERAL.kPIDLoopIdx, CARGO_ARM.ARM_F, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.config_kP(Constants.GENERAL.kPIDLoopIdx, CARGO_ARM.ARM_P, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.config_kI(Constants.GENERAL.kPIDLoopIdx, CARGO_ARM.ARM_I, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.config_kD(Constants.GENERAL.kPIDLoopIdx, CARGO_ARM.ARM_D, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.configMotionCruiseVelocity((int) CARGO_ARM.MOTION_MAGIC_CRUISE_VEL, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.configMotionAcceleration((int) CARGO_ARM.MOTION_MAGIC_ACCEL, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, GENERAL.kLongCANTimeoutMs));
      CTRE(masterTalon.configVelocityMeasurementWindow(64));
      zeroAbsolute();
    }
    masterTalon.selectProfileSlot(Constants.GENERAL.kSlotIdx, Constants.GENERAL.kPIDLoopIdx);
    CTRE(masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5));
    CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 160, GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 160, GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5, GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5, GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon
        .configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.GENERAL.kPIDLoopIdx, Constants.GENERAL.kLongCANTimeoutMs));
    masterTalon.setNeutralMode(NeutralMode.Brake);
    CTRE(masterTalon.configNominalOutputForward(0));
    CTRE(masterTalon.configNominalOutputReverse(0));
    CTRE(masterTalon.configPeakOutputForward(1.0));
    CTRE(masterTalon.configPeakOutputReverse(-1.0));
    CTRE(masterTalon.configVoltageCompSaturation(12.0));
    masterTalon.enableVoltageCompensation(true);
    CTRE(masterTalon.configVoltageMeasurementFilter(32, GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, GENERAL.kLongCANTimeoutMs));
    CTRE(masterTalon.configNeutralDeadband(0.0, GENERAL.kLongCANTimeoutMs));

    lastControlMode = ControlMode.PercentOutput;
    lastNeutralMode = NeutralMode.Brake;
    lastOutput = Double.NaN;


  }

  public void resetSlaveConfig() {
    CTRE(slaveTalon.configFactoryDefault(GENERAL.kLongCANTimeoutMs));
    slaveTalon.configAllSettings(new VictorSPXConfiguration());
    slaveTalon.setNeutralMode(NeutralMode.Brake);
    slaveTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5);
    slaveTalon.configNominalOutputForward(0);
    slaveTalon.configNominalOutputReverse(0);
    slaveTalon.configPeakOutputForward(1.0);
    slaveTalon.configPeakOutputReverse(-1.0);
    CTRE(slaveTalon.configVoltageCompSaturation(12.0, GENERAL.kLongCANTimeoutMs));
    slaveTalon.enableVoltageCompensation(true);
    CTRE(slaveTalon.configVoltageMeasurementFilter(32, GENERAL.kLongCANTimeoutMs));
    lastControlMode = ControlMode.PercentOutput;
    lastNeutralMode = NeutralMode.Brake;
    lastOutput = Double.NaN;
    slaveTalon.follow(masterTalon);
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
      slaveTalon.setNeutralMode(nMode);
    }
    masterTalon.set(mode, value, DemandType.ArbitraryFeedForward, arbFeed);
  }

  public void resetEncoder() {
    CTRE(masterTalon.setSelectedSensorPosition(0, Constants.GENERAL.kPIDLoopIdx, Constants.GENERAL.kMediumTimeoutMs));
  }

  public void updateSmartDash(boolean showRPM) {
    if (showRPM) {
      double rp = (((masterTalon.getSelectedSensorVelocity(0)) / 4096.0) * 60 * 10);
      maxRPM = maxRPM > rp ? maxRPM : rp;
      SmartDashboard.putNumber(side.toString() + " RPM", maxRPM);
    }
    SmartDashboard.putNumber(side.toString() + " Velocity", getSpeed());
    SmartDashboard.putNumber(side.toString() + " Error", getError());
    SmartDashboard.putNumber(side.toString() + " Master Output", masterTalon.getMotorOutputPercent());
    SmartDashboard.putNumber(side.toString() + " Position", getPosition());
  }

  public synchronized double getSpeed() {
    if (side == TalonLocation.CargoIntake) {
      return MkMath.nativeUnitsPer100MstoDegreesPerSec(masterTalon.getSelectedSensorVelocity(Constants.GENERAL.kPIDLoopIdx));
    } else {
      return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getSelectedSensorVelocity(Constants.GENERAL.kPIDLoopIdx));
    }
  }

  public synchronized double getPosition() {
    if (side == TalonLocation.CargoIntake) {
      return MkMath.nativeUnitsToDegrees(masterTalon.getSelectedSensorPosition(Constants.GENERAL.kPIDLoopIdx));
    } else {
      return MkMath.nativeUnitsToInches(masterTalon.getSelectedSensorPosition(Constants.GENERAL.kPIDLoopIdx));
    }
  }


  public synchronized double getCurrent() {
    return masterTalon.getOutputCurrent();
  }

  public double getAbsolutePosition() {
    return masterTalon.getSensorCollection().getPulseWidthPosition();
  }

  public void zeroAbsolute() {
    int pulseWidth = masterTalon.getSensorCollection().getPulseWidthPosition();
    if (pulseWidth > 0) {
      pulseWidth = pulseWidth & 0xFFF;
    } else {
      pulseWidth += (-Math.round(((double) pulseWidth / 4096) - 0.50)) * 4096;
    }
    CTRE(masterTalon.setSelectedSensorPosition(pulseWidth + (-CARGO_ARM.kBookEnd_0), Constants.GENERAL.kPIDLoopIdx,
        GENERAL.kMediumTimeoutMs));
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

    if (side == TalonLocation.Left_Drive || side == TalonLocation.Right_Drive) {
      resetEncoder();
      masterTalon.set(ControlMode.PercentOutput, 0.0);
      slaveTalon.set(ControlMode.PercentOutput, 1.0);
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

      resetEncoder();
      slaveTalon.set(ControlMode.PercentOutput, 0.0);
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

    } else if (side == TalonLocation.CargoIntake) {
      if (!isEncoderConnected()) {
        Logger.logError("Arm Encoder Not Connected");
        check = false;
      }
      for (ArmState state : ArmState.values()) {
        if (state != ArmState.ENABLE) {
          CargoArm.mArmState = state;
          CargoArm.getInstance().setIntakeRollers(-0.25);
          Timer.delay(2);
        }
      }
      resetMasterConfig();
      resetSlaveConfig();
    }

    return check;
  }

  public enum TalonLocation {
    Left_Drive, Right_Drive, HatchIntake, CargoIntake
  }

  public void CTRE(ErrorCode errorCode) {
    if (errorCode != ErrorCode.OK) {
      Logger.logError(errorCode.toString());
    }
  }
}
