package frc.robot.lib.drivers;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.math.MkMath;

public class MkTalon {
		public final TalonSRX masterTalon;
		public final VictorSPX slaveTalon;
		private TalonPosition side;
		private ControlMode lastControlMode = null;
		private double lastOutput = Double.NaN;
		private NeutralMode lastNeutralMode = null;

		/**
		 * @param master Talon with Encoder CAN ID
		 * @param slave  Follower Talon CAN ID
		 */
		public MkTalon(int master, int slave, TalonPosition side) {
				masterTalon = new TalonSRX(master);
				slaveTalon = new VictorSPX(slave);
				this.side = side;
				resetConfig();
		}

		public void resetConfig() {
				masterTalon.configFactoryDefault();
				masterTalon.configAllSettings(new TalonSRXConfiguration());
				slaveTalon.configFactoryDefault();
				slaveTalon.configAllSettings(new VictorSPXConfiguration());
				if (side.equals(TalonPosition.Left)) {
						masterTalon.config_kF(Constants.kPIDLoopIdx, Constants.LEFT_DRIVE_F);
				} else if (side.equals(TalonPosition.Right)) {
						masterTalon.config_kF(Constants.kPIDLoopIdx, Constants.RIGHT_DRIVE_F);
				}
				masterTalon.config_kP(Constants.kPIDLoopIdx, Constants.DRIVE_P);
				masterTalon.config_kI(Constants.kPIDLoopIdx, Constants.DRIVE_I);
				masterTalon.config_kD(Constants.kPIDLoopIdx, Constants.DRIVE_D);
				masterTalon.configMotionCruiseVelocity((int) Constants.MOTION_MAGIC_CRUISE_VEL);
				masterTalon.configMotionAcceleration((int) Constants.MOTION_MAGIC_ACCEL);
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
				masterTalon.configPeakOutputForward(1.0);
				masterTalon.configPeakOutputReverse(-1.0);
				slaveTalon.configNominalOutputForward(0);
				slaveTalon.configNominalOutputReverse(0);
				slaveTalon.configPeakOutputForward(1.0);
				slaveTalon.configPeakOutputReverse(-1.0);
				masterTalon.configVoltageCompSaturation(12);
				masterTalon.enableVoltageCompensation(true);
				masterTalon.configVoltageMeasurementFilter(32);
				slaveTalon.configVoltageCompSaturation(12);
				slaveTalon.enableVoltageCompensation(true);
				slaveTalon.configVoltageMeasurementFilter(32);
				masterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms);
				masterTalon.configVelocityMeasurementWindow(64);
				lastControlMode = ControlMode.PercentOutput;
				lastNeutralMode = NeutralMode.Brake;
				lastOutput = Double.NaN;
				slaveTalon.follow(masterTalon);
		}

		public boolean isEncoderConnected() {
				return masterTalon.getSensorCollection().getPulseWidthRiseToRiseUs() > 100;
		}

		public void set(ControlMode mode, double value, NeutralMode nMode) {
				set(mode, value, nMode, 0);
		}

		public void set(ControlMode mode, double value, NeutralMode nMode, double arbFeed) {
				if (lastOutput != value || lastControlMode != mode || lastNeutralMode != nMode) {
						lastNeutralMode = nMode;
						masterTalon.setNeutralMode(nMode);
						slaveTalon.setNeutralMode(nMode);
				}
				masterTalon.set(mode, value, DemandType.ArbitraryFeedForward, arbFeed);
		}

		public void resetEncoder() {
				masterTalon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		}

		public void setCoastMode() {
				lastNeutralMode = NeutralMode.Coast;
				masterTalon.setNeutralMode(NeutralMode.Coast);
				slaveTalon.setNeutralMode(NeutralMode.Coast);
		}

		public void setBrakeMode() {
				lastNeutralMode = NeutralMode.Brake;
				masterTalon.setNeutralMode(NeutralMode.Brake);
				slaveTalon.setNeutralMode(NeutralMode.Brake);
		}

		public void updateSmartDash() {
				SmartDashboard.putNumber(side.toString() + " Velocity", getSpeed());
				SmartDashboard.putNumber(side.toString() + " Error", getError());
				SmartDashboard.putNumber(side.toString() + " Master Output", masterTalon.getMotorOutputPercent());
				SmartDashboard.putNumber(side.toString() + " Position", getPosition());
		}

		public synchronized double getSpeed() {
				return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));
		}

		private double getError() {
				return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getClosedLoopError(Constants.kPIDLoopIdx));
		}

		public synchronized double getPosition() {
				return MkMath.nativeUnitsToInches(masterTalon.getSelectedSensorPosition(Constants.kPIDLoopIdx));
		}

		public enum TalonPosition {
				Left, Right
		}
}
