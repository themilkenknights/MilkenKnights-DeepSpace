package frc.robot.lib.drivers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.GENERAL;
import frc.robot.Constants.HATCH_ARM;
import frc.robot.Constants.TEST;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTime;
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
	private final int kShort = 0;
	private final int kSlot = GENERAL.kPIDLoopIdx;
	private TalonLoc mSide;
	private ControlMode lastControlMode = null;
	private double lastOutput, lastArbFeed, mMaxRPM = Double.NaN;
	private NeutralMode lastNeutralMode = null;
	private MkTime motorSafetyTimer = new MkTime();

	/**
	 * @param master Talon with Encoder CAN ID
	 * @param slave Follower Talon CAN ID
	 */
	public MkTalon(int master, int slave, TalonLoc mSide) {
		masterTalon = new TalonSRX(master);
		if (mSide == TalonLoc.Hatch_Arm) {
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
		CTRE(masterTalon.configAllSettings(new TalonSRXConfiguration()));
		CTRE(masterTalon.configFactoryDefault(kLong));
		CTRE(masterTalon.clearStickyFaults(kLong));
		masterTalon.selectProfileSlot(kSlot, kSlot);
		masterTalon.setNeutralMode(NeutralMode.Brake);
		CTRE(masterTalon.configNominalOutputForward(0.0, kLong));
		CTRE(masterTalon.configNominalOutputReverse(0.0, kLong));
		CTRE(masterTalon.configPeakOutputForward(GENERAL.kMaxNominalOutput, kLong));
		CTRE(masterTalon.configPeakOutputReverse(-GENERAL.kMaxNominalOutput, kLong));
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
			case Cargo_Arm:
				masterTalon.setSensorPhase(CARGO_ARM.ARM_SENSOR_PHASE);
				masterTalon.setInverted(CARGO_ARM.ARM_MASTER_DIRECTION);
				CTRE(masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kSlot, kLong));
				CTRE(masterTalon.config_kF(kSlot, CARGO_ARM.ARM_F, kLong));
				CTRE(masterTalon.config_kP(kSlot, CARGO_ARM.ARM_P, kLong));
				CTRE(masterTalon.config_kI(kSlot, CARGO_ARM.ARM_I, kLong));
				CTRE(masterTalon.config_kD(kSlot, CARGO_ARM.ARM_D, kLong));
				CTRE(masterTalon.config_IntegralZone(kSlot, 114, kLong));
				CTRE(masterTalon.configAllowableClosedloopError(kSlot, 6, kLong));
				CTRE(masterTalon.configMaxIntegralAccumulator(kSlot, 5000, kLong));
				CTRE(masterTalon.configMotionCruiseVelocity((int) CARGO_ARM.MOTION_MAGIC_CRUISE_VEL, kLong));
				CTRE(masterTalon.configMotionAcceleration((int) CARGO_ARM.MOTION_MAGIC_ACCEL, kLong));
				CTRE(masterTalon.configForwardSoftLimitThreshold((int) MkMath.angleToNativeUnits(CARGO_ARM.ARM_FORWARD_LIMIT), kLong));
				CTRE(masterTalon.configReverseSoftLimitThreshold((int) MkMath.angleToNativeUnits(CARGO_ARM.ARM_REVERSE_LIMIT), kLong));
				CTRE(masterTalon.configForwardSoftLimitEnable(true, kLong));
				CTRE(masterTalon.configReverseSoftLimitEnable(true, kLong));
				CTRE(masterTalon.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, CAN.kLeftCargoIntakeTalonID, kLong));
				break;
			case Hatch_Arm:
				masterTalon.setSensorPhase(HATCH_ARM.ARM_SENSOR_PHASE);
				masterTalon.setInverted(HATCH_ARM.ARM_MASTER_DIRECTION);
				CTRE(masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kSlot, kLong));
				CTRE(masterTalon.config_kF(kSlot, HATCH_ARM.ARM_F, kLong));
				CTRE(masterTalon.config_kP(kSlot, HATCH_ARM.ARM_P, kLong));
				CTRE(masterTalon.config_kI(kSlot, HATCH_ARM.ARM_I, kLong));
				CTRE(masterTalon.config_kD(kSlot, HATCH_ARM.ARM_D, kLong));
				CTRE(masterTalon.configMotionCruiseVelocity((int) HATCH_ARM.kMotionMagicCruiseVel, kLong));
				CTRE(masterTalon.configMotionAcceleration((int) HATCH_ARM.kMotionMagicAccel, kLong));
				CTRE(masterTalon.configForwardSoftLimitThreshold((int) MkMath.angleToNativeUnits(HATCH_ARM.ARM_FORWARD_LIMIT), kLong));
				CTRE(masterTalon.configReverseSoftLimitThreshold((int) MkMath.angleToNativeUnits(HATCH_ARM.ARM_REVERSE_LIMIT), kLong));
				CTRE(masterTalon.configForwardSoftLimitEnable(true, kLong));
				CTRE(masterTalon.configReverseSoftLimitEnable(true, kLong));
				CTRE(masterTalon.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteTalonSRX, LimitSwitchNormal.NormallyOpen, CAN.kHatchLimitSwitchTalonID, kLong));
				break;
			case Cargo_Intake:
				masterTalon.setInverted(CARGO_ARM.LEFT_INTAKE_DIRECTION);
				slaveVictor.setInverted(CARGO_ARM.RIGHT_INTAKE_DIRECTION);
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
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kLong));
				CTRE(masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kSlot, kLong));
				zeroEncoder();
				break;
			case Cargo_Arm:
			case Hatch_Arm:
				CTRE(masterTalon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_100Ms, kLong));
				CTRE(masterTalon.configVelocityMeasurementWindow(64, kLong));
				CTRE(masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 10));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 200, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 50, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 200, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 200, kLong));
				CTRE(masterTalon.configClearPositionOnLimitF(false, kLong));
				CTRE(masterTalon.configClearPositionOnLimitR(true, kLong));
				zeroEncoder();
				break;
			case Cargo_Intake:
				CTRE(masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 10));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 1000, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1000, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 1000, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 1000, kLong));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 1000, kLong));
				break;
			default:
				Logger.logError("Unknown Side");
				break;
		}

		//Set Slave Talons/Victors
		if (mSide == TalonLoc.Hatch_Arm) {
			CTRE(slaveTalon.configAllSettings(new TalonSRXConfiguration()));
			CTRE(slaveTalon.configFactoryDefault(kLong));
			CTRE(slaveTalon.clearStickyFaults(kLong));
			CTRE(slaveTalon.setControlFramePeriod(ControlFrame.Control_3_General, 50));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 500, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 500, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 500, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 500, kLong));
			CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 500, kLong));
			slaveTalon.setNeutralMode(NeutralMode.Brake);
			CTRE(slaveTalon.configNominalOutputForward(0, kLong));
			CTRE(slaveTalon.configNominalOutputReverse(0, kLong));
			CTRE(slaveTalon.configPeakOutputForward(GENERAL.kMaxNominalOutput, kLong));
			CTRE(slaveTalon.configPeakOutputReverse(-GENERAL.kMaxNominalOutput, kLong));
			CTRE(slaveTalon.configVoltageCompSaturation(12.0, kLong));
			slaveTalon.enableVoltageCompensation(true);
			CTRE(slaveTalon.configVoltageMeasurementFilter(32, kLong));
		} else {
			CTRE(slaveVictor.configAllSettings(new VictorSPXConfiguration()));
			CTRE(slaveVictor.configFactoryDefault(kLong));
			CTRE(slaveVictor.clearStickyFaults(kLong));
			slaveVictor.setNeutralMode(NeutralMode.Brake);
			slaveVictor.setControlFramePeriod(ControlFrame.Control_3_General, 5);
			CTRE(slaveVictor.configNominalOutputForward(0, kLong));
			CTRE(slaveVictor.configNominalOutputReverse(0, kLong));
			CTRE(slaveVictor.configPeakOutputForward(GENERAL.kMaxNominalOutput, kLong));
			CTRE(slaveVictor.configPeakOutputReverse(-GENERAL.kMaxNominalOutput, kLong));
			CTRE(slaveVictor.configVoltageCompSaturation(12.0, kLong));
			slaveVictor.enableVoltageCompensation(true);
			CTRE(slaveVictor.configVoltageMeasurementFilter(32, kLong));
			slaveVictor.follow(masterTalon);
		}

		if (mSide == TalonLoc.Cargo_Arm) {
			slaveVictor.setInverted(InvertType.OpposeMaster);
		}

		motorSafetyTimer.start(0.075);
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
		if (mode != lastControlMode || value != lastOutput || arbFeed != lastArbFeed || motorSafetyTimer.isDone()) {
			masterTalon.set(mode, value, DemandType.ArbitraryFeedForward, arbFeed);
			lastOutput = value;
			lastArbFeed = arbFeed;
			lastControlMode = mode;
			motorSafetyTimer.start(0.075);
		}
	}

	public synchronized void zeroEncoder() {
		if (mSide == TalonLoc.Left_Drive || mSide == TalonLoc.Right_Drive) {
			CTRE(masterTalon.setSelectedSensorPosition(0, kSlot, kShort));
		} else if (mSide == TalonLoc.Cargo_Arm) {
			CTRE(masterTalon.getSensorCollection()
					.syncQuadratureWithPulseWidth(CARGO_ARM.kBookEnd_0, CARGO_ARM.kBookEnd_1, CARGO_ARM.kCrossOverZero, CARGO_ARM.kOffset, kShort));
		} else if (mSide == TalonLoc.Hatch_Arm) {
			CTRE(masterTalon.getSensorCollection()
					.syncQuadratureWithPulseWidth(HATCH_ARM.kBookEnd_0, HATCH_ARM.kBookEnd_1, HATCH_ARM.kCrossOverZero, HATCH_ARM.kOffset, kShort));
		} else {
			Logger.logErrorWithTrace("Can't Zero Encoder: MkTalon Position - " + mSide.toString());
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
			case Cargo_Arm:
			case Hatch_Arm:
				return MkMath.nativeUnitsPer100MstoDegreesPerSec(masterTalon.getSelectedSensorVelocity(kSlot));
			case Left_Drive:
			case Right_Drive:
				return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getSelectedSensorVelocity(kSlot));
			default:
				Logger.logErrorWithTrace("Talon doesn't have encoder");
				return 0.0;
		}
	}

	public synchronized double getPosition() {
		switch (mSide) {
			case Cargo_Arm:
			case Hatch_Arm:
				return MkMath.nativeUnitsToDegrees(masterTalon.getSelectedSensorPosition(kSlot));
			case Left_Drive:
			case Right_Drive:
				return MkMath.nativeUnitsToInches(masterTalon.getSelectedSensorPosition(kSlot));
			default:
				Logger.logErrorWithTrace("Talon doesn't have encoder");
				return 0.0;
		}
	}

	/**
	 * Return the expected error value for the current mode Note that the method returns the deviation from target setpoint unlike the official getClosedLoopError() method.
	 * This method serves to limit CAN usage by using known setpoints to calculate error.
	 */
	public synchronized double getError() {
		switch (mSide) {
			case Cargo_Arm:
			case Hatch_Arm:
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
				Logger.logErrorWithTrace("Talon doesn't have encoder");
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
					Logger.logErrorWithTrace("FAILED - " + mSide.toString() + "Slave FAILED TO REACH REQUIRED SPEED OR POSITION");
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
					Logger.logErrorWithTrace("FAILED - " + mSide.toString() + "Master FAILED TO REACH REQUIRED SPEED OR POSITION");
					Logger.logMarker(mSide.toString() + " Master Test Failed - Vel: " + getSpeed() + " Pos: " + getPosition());
					check = false;
				} else {
					Logger.logMarker(mSide.toString() + " Master - Vel: " + getSpeed() + " Pos: " + getPosition());
				}

				if (currents.size() > 0) {
					Double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();
					if (!Util.allCloseTo(currents, average, TEST.kDriveCurrentEpsilon)) {
						Logger.logErrorWithTrace(mSide.toString() + " Currents varied!!!!!!!!!!!");
						check = true;
					}
				}

				if (positions.size() > 0) {
					Double average = positions.stream().mapToDouble(val -> val).average().getAsDouble();
					if (!Util.allCloseTo(positions, average, TEST.kDrivePosEpsilon)) {
						Logger.logErrorWithTrace(mSide.toString() + " Positions varied!!!!!!!!");
						check = true;
					}
				}

				if (velocities.size() > 0) {
					Double average = velocities.stream().mapToDouble(val -> val).average().getAsDouble();
					if (!Util.allCloseTo(velocities, average, TEST.kDriveVelEpsilon)) {
						Logger.logErrorWithTrace(mSide.toString() + " Velocities varied!!!!!!!!");
						check = true;
					}
				}
				break;
			case Cargo_Arm:
				if (!isEncoderConnected()) {
					Logger.logErrorWithTrace("Arm Encoder Not Connected");
					check = false;
				}
				for (CargoArmState state : CargoArmState.values()) {
					if (state != CargoArmState.ENABLE) {
						CargoArm.getInstance().setArmState(state);
						CargoArm.getInstance().setIntakeRollers(-0.25);
						Timer.delay(2.0);
					}
				}

				Timer.delay(1.0);
				CargoArm.getInstance().setArmState(CargoArmState.INTAKE);
				CargoArm.getInstance().setIntakeRollers(0.0);
				Timer.delay(1.0);
				masterTalon.set(ControlMode.PercentOutput, 0.0);
				slaveVictor.set(ControlMode.PercentOutput, -0.3);
				Timer.delay(1.0);
				double current = getCurrent();
				double vel = getSpeed();
				double pos = getPosition();
				currents.add(current);
				velocities.add(vel);
				positions.add(pos);
				if (vel < TEST.kMinCargoArmTestVel || pos > TEST.kMinCargoArmTestPos || current > TEST.kMinCargoArmTestCurrent) {
					Logger.logErrorWithTrace("FAILED - " + mSide.toString() + "Cargo Arm Slave FAILED TO REACH REQUIRED SPEED OR POSITION");
					Logger.logMarker(mSide.toString() + " Slave Test Failed - Vel: " + vel + " Pos: " + pos + " Current: " + current);
					check = false;
				} else {
					Logger.logMarker(mSide.toString() + " Slave - Vel: " + vel + " Pos: " + pos + " Current: " + current);
				}

				Timer.delay(1.0);
				CargoArm.getInstance().setArmState(CargoArmState.INTAKE);
				CargoArm.getInstance().setIntakeRollers(0.0);
				Timer.delay(1.0);
				masterTalon.set(ControlMode.PercentOutput, -0.3);
				slaveVictor.set(ControlMode.PercentOutput, 0.0);
				Timer.delay(1.0);
				current = getCurrent();
				vel = getSpeed();
				pos = getPosition();
				currents.add(current);
				velocities.add(vel);
				positions.add(pos);
				if (vel < TEST.kMinCargoArmTestVel || pos > TEST.kMinCargoArmTestPos || current > TEST.kMinCargoArmTestCurrent) {
					Logger.logErrorWithTrace("FAILED - " + mSide.toString() + "Cargo Arm Master FAILED TO REACH REQUIRED SPEED OR POSITION");
					Logger.logMarker(mSide.toString() + " Master Test Failed - Vel: " + vel + " Pos: " + pos + " Current: " + current);
					check = false;
				} else {
					Logger.logMarker(mSide.toString() + " Master - Vel: " + vel + " Pos: " + pos + " Current: " + current);
				}

				if (currents.size() > 0) {
					Double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();
					if (!Util.allCloseTo(currents, average, TEST.kCargoArmCurrentEpsilon)) {
						Logger.logErrorWithTrace(mSide.toString() + " Currents varied!!!!!!!!!!!");
						check = true;
					}
				}

				if (positions.size() > 0) {
					Double average = positions.stream().mapToDouble(val -> val).average().getAsDouble();
					if (!Util.allCloseTo(positions, average, TEST.kCargoArmPosEpsilon)) {
						Logger.logErrorWithTrace(mSide.toString() + " Positions varied!!!!!!!!");
						check = true;
					}
				}

				if (velocities.size() > 0) {
					Double average = velocities.stream().mapToDouble(val -> val).average().getAsDouble();
					if (!Util.allCloseTo(velocities, average, TEST.kCargoArmVelEpsilon)) {
						Logger.logErrorWithTrace(mSide.toString() + " Velocities varied!!!!!!!!");
						check = true;
					}
				}

				break;
			case Hatch_Arm:
				if (!isEncoderConnected()) {
					Logger.logErrorWithTrace("Arm Encoder Not Connected");
					check = false;
				}
				for (HatchMechanismState state : HatchMechanismState.values()) {
					HatchArm.getInstance().setHatchMechanismState(state);
					Timer.delay(2.0);
				}
				HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.GROUND_INTAKE);
				Timer.delay(1.0);
				masterTalon.set(ControlMode.PercentOutput, -0.3);
				Timer.delay(1.0);
				current = getCurrent();
				vel = getSpeed();
				pos = getPosition();
				if (vel < TEST.kHatchArmVel || current > TEST.kHatchArmCurrent || pos < TEST.kHatchArmPos) {
					Logger.logErrorWithTrace("FAILED - " + mSide.toString() + "Hatch FAILED TO REACH REQUIRED SPEED OR POSITION");
					Logger.logMarker(mSide.toString() + " Test Failed - Vel: " + vel + " Pos: " + pos + " Current: " + current);
					check = false;
				} else {
					Logger.logMarker(mSide.toString() + " - Vel: " + vel + " Pos: " + pos + " Current: " + current);
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
			Logger.logErrorWithTrace(errorCode.toString());
		}
	}

	@Override
	public String toString() {
		return "Output: " + masterTalon.getMotorOutputPercent() + " Current: " + masterTalon.getOutputCurrent() + (mSide != TalonLoc.Cargo_Intake ?
				" Pos: "
						+ getPosition() + " Vel: " + getSpeed() : " No Encoder");
	}

	/**
	 * Left Drive and Right Drive house the Talon, Victor, and SRX Mag encoder for each side of the drivetrain.
	 * The hatch Arm houses the ground hatch intake Talon, SRX Mag Encoder, and an unused Talon with a Breakout board with two limit switches.
	 * One limit switch is placed at the reverse hardstop for the ground intake arm, and the second is placed on the pneumatic peg arm to detect when the main arm is inside the target.
	 * The Cargo Arm houses the Talon, Victor, and SRX Mag encoder for the main cargo arm.
	 */
	public enum TalonLoc {
		Left_Drive, Right_Drive, Hatch_Arm, Cargo_Arm, Cargo_Intake
	}

}
