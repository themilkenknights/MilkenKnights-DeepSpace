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
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import java.util.ArrayList;

public class MkTalon {

	public final TalonSRX masterTalon, slaveTalon;
	private final VictorSPX slaveVictor;
	private final int kLong = GENERAL.kLongCANTimeoutMs;
	private final int kShort = GENERAL.kMediumTimeoutMs;
	private final int kSlot = GENERAL.kPIDLoopIdx;
	private TalonLoc mSide;
	private ControlMode lastControlMode = null;
	private double lastOutput, lastArbFeed, mMaxRPM = Double.NaN;
	private NeutralMode lastNeutralMode = null;
	//TODO see if caching setpoints is allowed

	/**
	 * @param master Talon with Encoder CAN ID
	 * @param slave Follower Talon CAN ID
	 */
	public MkTalon(int master, int slave, TalonLoc mSide) {
		masterTalon = new TalonSRX(master);
		if (mSide == TalonLoc.HatchArm) {
			slaveVictor = null;
			slaveTalon = new TalonSRX(slave);
		} else {
			slaveVictor = new VictorSPX(slave);
			slaveTalon = null;
		}
		this.mSide = mSide;
		resetConfig();
	}


	/**
	 * Configures all Talon/Victor Configs at Startup based on the position This method is meant to contain all the mess in one place and ensure that each parameter is set
	 * correctly. Errors will appear on the Driver Station and will be logged to disk if a config() method return an error Every position except for the Hatch Arm has a
	 * Talon and a Victor. The Hatch Arm has two Talons.
	 */
	public synchronized void resetConfig() {
		lastControlMode = ControlMode.PercentOutput;
		lastNeutralMode = NeutralMode.Brake;
		lastOutput = Double.NaN;

		CTRE(masterTalon.configFactoryDefault(kLong));
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

		switch (mSide) {
			case Left_Drive:
				CTRE(masterTalon.config_kF(kSlot, Constants.DRIVE.kLeftDriveF, kLong));
				masterTalon.setInverted(Constants.DRIVE.kLeftMasterInvert);
				slaveVictor.setInverted(Constants.DRIVE.kLeftSlaveInvert);
				masterTalon.setSensorPhase(Constants.DRIVE.kLeftSensorInvert);
				break;
			case Right_Drive:
				CTRE(masterTalon.config_kF(kSlot, Constants.DRIVE.kRightDriveF, kLong));
				masterTalon.setInverted(Constants.DRIVE.KRightMasterInvert);
				slaveVictor.setInverted(Constants.DRIVE.kRightSlaveInvert);
				masterTalon.setSensorPhase(Constants.DRIVE.kRightSensorInvert);
				break;
			case CargoArm:
				masterTalon.setSensorPhase(CARGO_ARM.ARM_SENSOR_PHASE);
				masterTalon.setInverted(CARGO_ARM.ARM_MASTER_DIRECTION);
				slaveVictor.setInverted(CARGO_ARM.ARM_SLAVE_DIRECTION);
				CTRE(masterTalon.config_kF(kSlot, CARGO_ARM.ARM_F, kLong));
				CTRE(masterTalon.config_kP(kSlot, CARGO_ARM.ARM_P, kLong));
				CTRE(masterTalon.config_kI(kSlot, CARGO_ARM.ARM_I, kLong));
				CTRE(masterTalon.config_kD(kSlot, CARGO_ARM.ARM_D, kLong));
				CTRE(masterTalon.configMotionCruiseVelocity((int) CARGO_ARM.MOTION_MAGIC_CRUISE_VEL, kLong));
				CTRE(masterTalon.configMotionAcceleration((int) CARGO_ARM.MOTION_MAGIC_ACCEL, kLong));
				CTRE(masterTalon.configForwardSoftLimitThreshold((int) CARGO_ARM.ARM_FORWARD_LIMIT, kLong));
				CTRE(masterTalon.configReverseSoftLimitThreshold((int) CARGO_ARM.ARM_REVERSE_LIMIT, kLong));
				break;
			case HatchArm:
				masterTalon.setSensorPhase(HATCH_ARM.ARM_SENSOR_PHASE);
				masterTalon.setInverted(HATCH_ARM.ARM_MASTER_DIRECTION);
				slaveVictor.setInverted(HATCH_ARM.ARM_SLAVE_DIRECTION);
				CTRE(masterTalon.config_kF(kSlot, HATCH_ARM.ARM_F, kLong));
				CTRE(masterTalon.config_kP(kSlot, HATCH_ARM.ARM_P, kLong));
				CTRE(masterTalon.config_kI(kSlot, HATCH_ARM.ARM_I, kLong));
				CTRE(masterTalon.config_kD(kSlot, HATCH_ARM.ARM_D, kLong));
				CTRE(masterTalon.configMotionCruiseVelocity((int) HATCH_ARM.kMotionMagicCruiseVel, kLong));
				CTRE(masterTalon.configMotionAcceleration((int) HATCH_ARM.kMotionMagicAccel, kLong));
				CTRE(masterTalon.configForwardSoftLimitThreshold((int) HATCH_ARM.ARM_FORWARD_LIMIT, kLong));
				CTRE(masterTalon.configReverseSoftLimitThreshold((int) HATCH_ARM.ARM_REVERSE_LIMIT, kLong));
				break;
			case CargoIntake:
				masterTalon.setInverted(CARGO_ARM.LEFT_INTAKE_DIRECTION);
				slaveVictor.setInverted(CARGO_ARM.RIGHT_INTAKE_DIRECTION);
				CTRE(slaveTalon.setControlFramePeriod(ControlFrame.Control_3_General, 1000));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kLong));
				break;
			default:
				Logger.logError("Unknown Side");
				break;
		}

		switch (mSide) {
			case Left_Drive:
			case Right_Drive:
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
				zeroEncoder();
				break;
			case CargoArm:
			case HatchArm:
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
				CTRE(masterTalon.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, kLong));
				zeroEncoder();
				break;
			case CargoIntake:
				masterTalon.setInverted(CARGO_ARM.LEFT_INTAKE_DIRECTION);
				slaveVictor.setInverted(CARGO_ARM.RIGHT_INTAKE_DIRECTION);
				CTRE(slaveTalon.setControlFramePeriod(ControlFrame.Control_3_General, 1000));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, kLong));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kLong));
				break;
			default:
				Logger.logError("Unknown Side");
				break;
		}

		if (mSide == TalonLoc.HatchArm) {
			CTRE(masterTalon.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0, 0, kLong));
			//TODO Ensure that I want to zero at limit
			CTRE(slaveTalon.configFactoryDefault(kLong));
			CTRE(slaveTalon.setControlFramePeriod(ControlFrame.Control_3_General, 1000));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kLong));
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
		set(mode, value, nMode, 0.0);
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

	public synchronized void zeroEncoder() {
		if (mSide == TalonLoc.Left_Drive || mSide == TalonLoc.Right_Drive) {
			CTRE(masterTalon.setSelectedSensorPosition(0, kSlot, kShort));
		} else if (mSide == TalonLoc.CargoArm) {
			CTRE(masterTalon.getSensorCollection()
					.syncQuadratureWithPulseWidth(CARGO_ARM.kBookEnd_0, CARGO_ARM.kBookEnd_1, CARGO_ARM.kCrossOverZero, CARGO_ARM.kOffset, kShort));
		} else if (mSide == TalonLoc.HatchArm) {
			CTRE(masterTalon.getSensorCollection()
					.syncQuadratureWithPulseWidth(HATCH_ARM.kBookEnd_0, HATCH_ARM.kBookEnd_1, HATCH_ARM.kCrossOverZero, HATCH_ARM.kOffset, kShort));
		} else {
			Logger.logCriticalError("Can't Zero Encoder: MkTalon Position - " + mSide.toString());
		}
	}

	public synchronized void updateSmartDash(boolean showRPM) {
		if (showRPM) {
			double rp = (((masterTalon.getSelectedSensorVelocity(0)) / 4096.0) * 60 * 10);
			mMaxRPM = mMaxRPM > rp ? mMaxRPM : rp;
			SmartDashboard.putNumber(mSide.toString() + " RPM", mMaxRPM);
		}
		SmartDashboard.putNumber(mSide.toString() + " Velocity", getSpeed());
		SmartDashboard.putNumber(mSide.toString() + " Error", getError());
		SmartDashboard.putNumber(mSide.toString() + " Master Output", masterTalon.getMotorOutputPercent());
		SmartDashboard.putNumber(mSide.toString() + " Position", getPosition());
	}

	public synchronized double getSpeed() {
		switch (mSide) {
			case CargoArm:
			case HatchArm:
				return MkMath.nativeUnitsPer100MstoDegreesPerSec(masterTalon.getSelectedSensorVelocity(kSlot));
			case Left_Drive:
			case Right_Drive:
				return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getSelectedSensorVelocity(kSlot));
			default:
				Logger.logCriticalError("Talon doesn't have encoder");
				return 0.0;
		}
	}

	public synchronized double getPosition() {
		switch (mSide) {
			case CargoArm:
			case HatchArm:
				return MkMath.nativeUnitsToDegrees(masterTalon.getSelectedSensorPosition(kSlot));
			case Left_Drive:
			case Right_Drive:
				return MkMath.nativeUnitsToInches(masterTalon.getSelectedSensorPosition(kSlot));
			default:
				Logger.logCriticalError("Talon doesn't have encoder");
				return 0.0;
		}
	}

	/**
	 * Return the expected error value for the current mode Note that the method returns the deviation from target setpoint unlike the official getClosedLoopError() method.
	 * This method serves to limit CAN usage by using known setpoints to calculate error. TODO Verify getError() Method
	 */
	public synchronized double getError() {
		switch (mSide) {
			case CargoArm:
			case HatchArm:
				if (lastControlMode == ControlMode.MotionMagic) {
					return MkMath.nativeUnitsToDegrees(lastOutput) - getPosition();
				} else {
					return 0.0;
				}
			case Left_Drive:
			case Right_Drive:
				if (lastControlMode == ControlMode.Velocity) {
					return MkMath.nativeUnitsPer100MstoInchesPerSec(lastOutput) - getSpeed();
				} else if (lastControlMode == ControlMode.MotionMagic) {
					return MkMath.nativeUnitsToInches(lastOutput) - getPosition();
				} else {
					return 0.0;
				}
			default:
				Logger.logCriticalError("Talon doesn't have encoder");
				return 0.0;
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

		switch (mSide) {
			case Left_Drive:
			case Right_Drive:
				zeroEncoder();
				masterTalon.set(ControlMode.PercentOutput, 0.0);
				slaveVictor.set(ControlMode.PercentOutput, 1.0);
				Timer.delay(5.0);
				currents.add(getCurrent());
				velocities.add(getSpeed());
				positions.add(getPosition());
				if (getPosition() < TEST.kMinDriveTestPos || getSpeed() < TEST.kMinDriveTestVel
						|| masterTalon.getOutputCurrent() > TEST.kMinDriveTestCurrent) {
					Logger.logCriticalError("FAILED - " + mSide.toString() + "Slave FAILED TO REACH REQUIRED SPEED OR POSITION");
					Logger.logMarker(mSide.toString() + " Slave Test Failed - Vel: " + getSpeed() + " Pos: " + getPosition());
					check = false;
				} else {
					Logger.logMarker(mSide.toString() + " Slave - Vel: " + getSpeed() + " Pos: " + getPosition());
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
					Logger.logCriticalError("FAILED - " + mSide.toString() + "Master FAILED TO REACH REQUIRED SPEED OR POSITION");
					Logger.logMarker(mSide.toString() + " Master Test Failed - Vel: " + getSpeed() + " Pos: " + getPosition());
					check = false;
				} else {
					Logger.logMarker(mSide.toString() + " Master - Vel: " + getSpeed() + " Pos: " + getPosition());
				}

				if (currents.size() > 0) {
					Double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();
					if (!Util.allCloseTo(currents, average, TEST.kDriveCurrentEpsilon)) {
						Logger.logCriticalError(mSide.toString() + " Currents varied!!!!!!!!!!!");
						check = true;
					}
				}

				if (positions.size() > 0) {
					Double average = positions.stream().mapToDouble(val -> val).average().getAsDouble();
					if (!Util.allCloseTo(positions, average, TEST.kDrivePosEpsilon)) {
						Logger.logCriticalError(mSide.toString() + " Positions varied!!!!!!!!");
						check = true;
					}
				}

				if (velocities.size() > 0) {
					Double average = velocities.stream().mapToDouble(val -> val).average().getAsDouble();
					if (!Util.allCloseTo(velocities, average, TEST.kDriveVelEpsilon)) {
						Logger.logCriticalError(mSide.toString() + " Velocities varied!!!!!!!!");
						check = true;
					}
				}
				break;
			case CargoArm:
				if (!isEncoderConnected()) {
					Logger.logCriticalError("Arm Encoder Not Connected");
					check = false;
				}
				for (CargoArmState state : CargoArmState.values()) {
					if (state != CargoArmState.ENABLE) {
						CargoArm.getInstance().setArmState(state);
						CargoArm.getInstance().setIntakeRollers(-0.25);
						Timer.delay(2.0);
					}
				}
				break;
			case HatchArm:
				if (!isEncoderConnected()) {
					Logger.logCriticalError("Arm Encoder Not Connected");
					check = false;
				}
				for (HatchMechanismState state : HatchMechanismState.values()) {
					HatchArm.getInstance().setHatchMechanismState(state);
					Timer.delay(2.0);
				}
				break;
			default:
				Logger.logError("Can't Check System!!!");
				break;
		}

		resetConfig();
		return check;
	}

	private void CTRE(ErrorCode errorCode) {
		if (errorCode != ErrorCode.OK) {
			Logger.logCriticalError(errorCode.toString());
		}
	}

	@Override
	public String toString() {
		return "Output: " + masterTalon.getMotorOutputPercent() + " Current: " + masterTalon.getOutputCurrent() + (mSide != TalonLoc.CargoIntake ?
				" Pos: "
						+ getPosition() + " Vel: " + getSpeed() : " No Encoder");
	}

	public enum TalonLoc {
		Left_Drive, Right_Drive, HatchArm, CargoArm, CargoIntake
	}
}
