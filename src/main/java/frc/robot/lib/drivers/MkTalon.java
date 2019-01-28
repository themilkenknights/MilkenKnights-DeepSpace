package frc.robot.lib.drivers;

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
import frc.robot.Constants.TEST;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.Logger;

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
    return MkMath.nativeUnitsPer100MstoInchesPerSec(
        masterTalon.getClosedLoopError(Constants.GENERAL.kPIDLoopIdx));
  }

  public void resetMasterConfig() {
    masterTalon.configFactoryDefault();
    masterTalon.configAllSettings(new TalonSRXConfiguration());
    if (side.equals(TalonLocation.Left_Drive)) {
      masterTalon.config_kF(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kLeftDriveF);
    } else if (side.equals(TalonLocation.Right_Drive)) {
      masterTalon.config_kF(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kRightDriveF);
    }
    masterTalon.config_kP(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kDriveP);
    masterTalon.config_kI(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kDriveI);
    masterTalon.config_kD(Constants.GENERAL.kPIDLoopIdx, Constants.DRIVE.kDriveD);
    masterTalon.configMotionCruiseVelocity((int) Constants.DRIVE.kMotionMagicCruiseNativeVel);
    masterTalon.configMotionAcceleration((int) Constants.DRIVE.kMotionMagicNativeAccel);
    masterTalon.selectProfileSlot(Constants.GENERAL.kSlotIdx, Constants.GENERAL.kPIDLoopIdx);
    masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5);
    masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5);
    masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5);
    masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 160);
    masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 160);
    masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 5);
    masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 5);
    masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        Constants.GENERAL.kPIDLoopIdx, Constants.GENERAL.kTimeoutMs);
    masterTalon.setNeutralMode(NeutralMode.Brake);
    masterTalon.configNominalOutputForward(0);
    masterTalon.configNominalOutputReverse(0);
    masterTalon.configPeakOutputForward(1.0);
    masterTalon.configPeakOutputReverse(-1.0);
    masterTalon.configVoltageCompSaturation(12.0);
    masterTalon.enableVoltageCompensation(true);
    masterTalon.configVoltageMeasurementFilter(32);
    masterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms);
    masterTalon.configVelocityMeasurementWindow(32);
    lastControlMode = ControlMode.PercentOutput;
    lastNeutralMode = NeutralMode.Brake;
    lastOutput = Double.NaN;
  }

  public void resetSlaveConfig() {
    slaveTalon.configFactoryDefault();
    slaveTalon.configAllSettings(new VictorSPXConfiguration());
    slaveTalon.setNeutralMode(NeutralMode.Brake);
    slaveTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5);
    slaveTalon.configNominalOutputForward(0);
    slaveTalon.configNominalOutputReverse(0);
    slaveTalon.configPeakOutputForward(1.0);
    slaveTalon.configPeakOutputReverse(-1.0);
    slaveTalon.configVoltageCompSaturation(12.0);
    slaveTalon.enableVoltageCompensation(true);
    slaveTalon.configVoltageMeasurementFilter(32);
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
    masterTalon.setSelectedSensorPosition(0, Constants.GENERAL.kPIDLoopIdx, Constants.GENERAL.kTimeoutMs);
  }

  public void updateSmartDash() {
    double rp = (((masterTalon.getSelectedSensorVelocity(0)) / 4096.0) * 60 * 10);
    maxRPM = maxRPM > rp ? maxRPM : rp;
    SmartDashboard.putNumber(side.toString() + " RPM", maxRPM);
    SmartDashboard.putNumber(side.toString() + " Velocity", getSpeed());
    SmartDashboard.putNumber(side.toString() + " Error", getError());
    SmartDashboard.putNumber(side.toString() + " Master Output", masterTalon.getMotorOutputPercent());
    SmartDashboard.putNumber(side.toString() + " Position", getPosition());
  }

  public synchronized double getSpeed() {
    return MkMath.nativeUnitsPer100MstoInchesPerSec(
        masterTalon.getSelectedSensorVelocity(Constants.GENERAL.kPIDLoopIdx));
  }

  public synchronized double getPosition() {
    return MkMath
        .nativeUnitsToInches(masterTalon.getSelectedSensorPosition(Constants.GENERAL.kPIDLoopIdx));
  }

  public boolean checkSystem() {
    boolean check = true;
    if (side == TalonLocation.Left_Drive || side == TalonLocation.Right_Drive) {
      resetEncoder();
      masterTalon.set(ControlMode.PercentOutput, 0.0);
      slaveTalon.set(ControlMode.PercentOutput, 1.0);
      Timer.delay(2.0);
      if (getPosition() < Constants.TEST.kMinTestPos || getSpeed() < Constants.TEST.kMinTestVel
          || masterTalon.getOutputCurrent() > TEST.kMinTestCurrent) {
        Logger.logError("FAILED - " + side.toString() + "Slave FAILED TO REACH REQUIRED SPEED OR POSITION");
        Logger.logMarker(side.toString() + " Slave Test Failed - Vel: " + getSpeed() + " Pos: " + getPosition());
        check = false;
      } else {
        Logger.logMarker(side.toString() + " Slave - Vel: " + getSpeed() + " Pos: " + getPosition());
      }

      resetEncoder();
      slaveTalon.set(ControlMode.PercentOutput, 0.0);
      masterTalon.set(ControlMode.PercentOutput, 1.0);
      Timer.delay(2.0);
      if (getPosition() < Constants.TEST.kMinTestPos || getSpeed() < Constants.TEST.kMinTestVel
          || masterTalon.getOutputCurrent() > TEST.kMinTestCurrent) {
        Logger.logError("FAILED - " + side.toString() + "Master FAILED TO REACH REQUIRED SPEED OR POSITION");
        Logger.logMarker(side.toString() + " Master Test Failed - Vel: " + getSpeed() + " Pos: " + getPosition());
        check = false;
      } else {
        Logger.logMarker(side.toString() + " Master - Vel: " + getSpeed() + " Pos: " + getPosition());
      }
    }
    return check;
  }

  public enum TalonLocation {
    Left_Drive, Right_Drive, HatchIntake, CargoIntake
  }
}
