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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.MkMath;

public class MkTalon {

	public final TalonSRX masterTalon;
	public final VictorSPX slaveTalon;
	private TalonPosition side;
	private double maxRPM = 0;
	private NeutralMode talonMode;

	/**
	 * @param master Talon with Encoder CAN ID
	 * @param slave Follower Talon CAN ID
	 */
	public MkTalon(int master, int slave, TalonPosition side) {
		masterTalon = new TalonSRX(master);
		slaveTalon = new VictorSPX(slave);

		this.side = side;
		masterTalon.configFactoryDefault();
		masterTalon.configAllSettings(new TalonSRXConfiguration());

		slaveTalon.configFactoryDefault();

		resetConfig();
		configMotionMagic();
		talonMode = NeutralMode.Brake;
	}

	public void setPIDF() {
		if (side.equals(TalonPosition.Left)) {
			masterTalon.config_kF(Constants.kPIDLoopIdx, Constants.LEFT_DRIVE_F);
		} else if (side.equals(TalonPosition.Right)) {
			masterTalon.config_kF(Constants.kPIDLoopIdx, Constants.RIGHT_DRIVE_F);
		}

		masterTalon.config_kP(Constants.kPIDLoopIdx, Constants.DRIVE_P);
		masterTalon.config_kI(Constants.kPIDLoopIdx, Constants.DRIVE_I);
		masterTalon.config_kD(Constants.kPIDLoopIdx, Constants.DRIVE_D);
	}

	private void configMotionMagic() {
		if (side == TalonPosition.Left) {
			masterTalon.config_kF(Constants.kPIDLoopIdx, Constants.LEFT_DRIVE_F);
		} else {
			masterTalon.config_kF(Constants.kPIDLoopIdx, Constants.RIGHT_DRIVE_F);
		}
		masterTalon.config_kP(Constants.kPIDLoopIdx, Constants.DRIVE_P);
		masterTalon.config_kI(Constants.kPIDLoopIdx, Constants.DRIVE_I);
		masterTalon.config_kD(Constants.kPIDLoopIdx, Constants.DRIVE_D);
		masterTalon.configMotionCruiseVelocity((int) Constants.MOTION_MAGIC_CRUISE_VEL);
		masterTalon.configMotionAcceleration((int) Constants.MOTION_MAGIC_ACCEL);
	}

	public void configTeleopVelocity() {
		if (side.equals(TalonPosition.Left)) {
			masterTalon.config_kF(Constants.kPIDLoopIdx, Constants.LEFT_DRIVE_F);
		} else if (side.equals(TalonPosition.Right)) {
			masterTalon.config_kF(Constants.kPIDLoopIdx, Constants.RIGHT_DRIVE_F);
		}
		masterTalon.config_kP(Constants.kPIDLoopIdx, Constants.TELEOP_DRIVE_P);
		masterTalon.config_kI(Constants.kPIDLoopIdx, Constants.TELEOP_DRIVE_I);
		masterTalon.config_kD(Constants.kPIDLoopIdx, Constants.TELEOP_DRIVE_D);
	}

	public void resetConfig() {
		masterTalon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5);
		masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5);
		masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5);
		masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 20);
		masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 20);
		masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 20);
		masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 20);

		masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		masterTalon.setNeutralMode(NeutralMode.Brake);
		slaveTalon.setNeutralMode(NeutralMode.Brake);

		masterTalon.configNominalOutputForward(0);
		masterTalon.configNominalOutputReverse(0);
		masterTalon.configPeakOutputForward(0.5);
		masterTalon.configPeakOutputReverse(-0.5);

		slaveTalon.configNominalOutputForward(0);
		slaveTalon.configNominalOutputReverse(0);
		slaveTalon.configPeakOutputForward(0.5);
		slaveTalon.configPeakOutputReverse(-0.5);

		masterTalon.configVoltageCompSaturation(12);
		masterTalon.enableVoltageCompensation(true);
		masterTalon.configVoltageMeasurementFilter(32);

		slaveTalon.configVoltageCompSaturation(12);
		slaveTalon.enableVoltageCompensation(true);
		slaveTalon.configVoltageMeasurementFilter(32);

		masterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
		masterTalon.configVelocityMeasurementWindow(64);

		slaveTalon.follow(masterTalon);
	}

	public boolean isEncoderConnected() {
		return masterTalon.getSensorCollection().getPulseWidthRiseToRiseUs() > 100;
	}

	public synchronized double getPosition() {
		return MkMath.nativeUnitsToInches(masterTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx));
	}

	public synchronized double getSpeed() {
		return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
	}

	private double getError() {
		return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getClosedLoopError(Constants.kPIDLoopIdx));
	}

	private double InchesPerSecToUnitsPer100Ms(double vel) {
		return MkMath.InchesToNativeUnits(vel) / 10;
	}

	public void set(ControlMode mode, double value, boolean nMode) {
		set(mode, value, nMode, 0);
	}

	public void set(ControlMode mode, double value, boolean nMode, double arbFeed) {
		if (talonMode != (nMode ? NeutralMode.Brake : NeutralMode.Coast)) {
			talonMode = nMode ? NeutralMode.Brake : NeutralMode.Coast;
			masterTalon.setNeutralMode(nMode ? NeutralMode.Brake : NeutralMode.Coast);
			slaveTalon.setNeutralMode(nMode ? NeutralMode.Brake : NeutralMode.Coast);
		}
		masterTalon.set(mode, value, DemandType.ArbitraryFeedForward, arbFeed);
	}

	public void resetEncoder() {
		masterTalon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
	}

	public void setCoastMode() {
		talonMode = NeutralMode.Coast;
		masterTalon.setNeutralMode(NeutralMode.Coast);
		slaveTalon.setNeutralMode(NeutralMode.Coast);
	}

	public void setBrakeMode() {
		talonMode = NeutralMode.Brake;
		masterTalon.setNeutralMode(NeutralMode.Brake);
		slaveTalon.setNeutralMode(NeutralMode.Brake);
	}

	public void updateSmartDash() {
		SmartDashboard.putNumber(side.toString() + " Velocity", getSpeed());
		SmartDashboard.putNumber(side.toString() + " Error", getError());
		SmartDashboard.putNumber(side.toString() + " Master Output", masterTalon.getMotorOutputPercent());
		SmartDashboard.putNumber(side.toString() + " Position", getPosition());
	}

	public enum TalonPosition {
		Left, Right
	}

}
