package frc.robot.misc;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.CONFIG;
import frc.robot.Constants.DRIVE;
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

	private static ArrayList<Double> currents = new ArrayList<>();
	private static ArrayList<Double> velocities = new ArrayList<>();
	private static ArrayList<Double> positions = new ArrayList<>();
	public final TalonSRX masterTalon, slaveTalon;
	public final VictorSPX slaveVictor;
	private final int kShort = 20;
	private final int kLong = 50;
	private final double motorTimer = GENERAL.kMotorSafetyTimer;
	public PigeonIMU mPigeon;
	private TalonLoc mSide;
	private ControlMode lastControlMode = null;
	private double lastOutput, lastArbFeed = Double.NaN;
	private NeutralMode lastNeutralMode = null;
	private DemandType lastDemandType = null;
	private MkTime motorSafetyTimer = new MkTime();
	private NetworkTableEntry mVel, mPos, mError, mOutput;

	/**
	 * @param master Talon with Encoder CAN ID
	 * @param slave Follower Talon CAN ID
	 */
	public MkTalon(int master, int slave, TalonLoc mSide, ShuffleboardTab mTab) {
		masterTalon = new TalonSRX(master);
		if (mSide == TalonLoc.Hatch_Arm || mSide == TalonLoc.Cargo_Intake) {
			slaveVictor = null;
			slaveTalon = new TalonSRX(slave);
		} else {
			slaveVictor = new VictorSPX(slave);
			slaveTalon = null;
		}
		this.mSide = mSide;
		if (mSide != TalonLoc.Cargo_Intake) {
			mVel = mTab.add(mSide.toString() + "Vel", 0.0).getEntry();
			mPos = mTab.add(mSide.toString() + " Pos", 0.0).getEntry();
			mError = mTab.add(mSide.toString() + " Err", 0.0).getEntry();
			mOutput = mTab.add(mSide.toString() + " Out", 0.0).getEntry();
		}
		resetConfig();
	}

	/**
	 * Configures all Talon/Victor Configs at startup based on the talon position
	 *
	 * <p>
	 * This method is meant to contain the mess in one place and ensure that each parameter is set
	 * correctly.
	 *
	 * <p>
	 * All of the persistent configs are stored in {@link Constants.CONFIG}. These primarily configure
	 * PID/Limit Switch/Encoder settings.
	 *
	 * <p>
	 * Errors will appear on the Driver Station and will be logged to disk if a config() method return
	 * an error.
	 */
	public synchronized void resetConfig() {
		lastControlMode = ControlMode.PercentOutput;
		lastNeutralMode = NeutralMode.Brake;
		lastOutput = Double.NaN;
		CTRE(masterTalon.clearStickyFaults(kShort));
		CTRE(masterTalon.configFactoryDefault(kShort));
		CTRE(masterTalon.configAllSettings(CONFIG.kConfigs.get(mSide), kLong));
		masterTalon.setNeutralMode(NeutralMode.Brake);
		masterTalon.enableVoltageCompensation(true);
		switch (mSide) {
			case Left:
				masterTalon.setInverted(Constants.DRIVE.kLeftInvert);
				slaveVictor.setInverted(InvertType.FollowMaster);
				masterTalon.setSensorPhase(Constants.DRIVE.kLeftSensorInvert);
				break;
			case Right:
				masterTalon.setInverted(Constants.DRIVE.KRightInvert);
				slaveVictor.setInverted(InvertType.FollowMaster);
				masterTalon.setSensorPhase(Constants.DRIVE.kRightSensorInvert);
				break;
			case Cargo_Arm:
				masterTalon.setInverted(CARGO_ARM.kCargoArmDirection);
				masterTalon.setSensorPhase(CARGO_ARM.kCargoArmSensorPhase);
				break;
			case Hatch_Arm:
				masterTalon.setSensorPhase(HATCH_ARM.kHatchArmSensorPhase);
				masterTalon.setInverted(HATCH_ARM.kHatchArmMasterDirection);
				break;
			case Cargo_Intake:
				masterTalon.setInverted(CARGO_ARM.kLeftIntakeDirection);
				slaveTalon.setInverted(CARGO_ARM.kRightIntakeDirection);
				break;
			default:
				Logger.logError("Unknown Side");
				break;
		}
		switch (mSide) {
			case Right:
				CTRE(masterTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 10, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 10, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 10, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10, kShort));
				masterTalon.selectProfileSlot(CONFIG.kDistanceSlot, CONFIG.kPIDPrimary);
				masterTalon.selectProfileSlot(CONFIG.kTurningSlot, CONFIG.kPIDAuxilliaryTurn);
			case Left:
				CTRE(masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kShort));
				zeroEncoder();
				break;
			case Cargo_Arm:
			case Hatch_Arm:
				CTRE(masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 20, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 10, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 50, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 50, kShort));
				zeroEncoder();
				break;
			case Cargo_Intake:
				CTRE(masterTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 5, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100, kShort));
				CTRE(masterTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 5, kShort));
				break;
			default:
				Logger.logError("Unknown Side");
				break;
		}

		// Set Slave Talons/Victors
		if (mSide == TalonLoc.Hatch_Arm || mSide == TalonLoc.Cargo_Intake) {
			CTRE(slaveTalon.clearStickyFaults(kShort));
			CTRE(slaveTalon.configFactoryDefault(kLong));
			CTRE(slaveTalon.configAllSettings(CONFIG.kConfigs.get(mSide)));
			slaveTalon.enableVoltageCompensation(true);
			slaveTalon.setNeutralMode(NeutralMode.Brake);
			if (mSide == TalonLoc.Hatch_Arm) {
				CTRE(slaveTalon.setControlFramePeriod(ControlFrame.Control_3_General, 10));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 3, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100, kShort));
			} else if (mSide == TalonLoc.Cargo_Intake) {
				CTRE(slaveTalon.setControlFramePeriod(ControlFrame.Control_3_General, 5));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 3, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 100, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 100, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 100, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100, kShort));
				CTRE(slaveTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 100, kShort));
				slaveTalon.follow(masterTalon);
			}
		} else {
			CTRE(slaveVictor.clearStickyFaults(kShort));
			CTRE(slaveVictor.configFactoryDefault(kLong));
			CTRE(slaveVictor.configVoltageCompSaturation(12.0, kShort));
			CTRE(slaveVictor.configVoltageMeasurementFilter(32, kShort));
			CTRE(slaveVictor.configNeutralDeadband(0.0, kShort));
			CTRE(slaveVictor.configPeakOutputForward(GENERAL.kMaxNominalOutput, kShort));
			CTRE(slaveVictor.configPeakOutputReverse(-GENERAL.kMaxNominalOutput, kShort));
			slaveVictor.enableVoltageCompensation(true);
			slaveVictor.setNeutralMode(NeutralMode.Brake);
			slaveVictor.setControlFramePeriod(ControlFrame.Control_3_General, 5);
			slaveVictor.follow(masterTalon);
		}
		if (mSide == TalonLoc.Cargo_Arm) {
			slaveVictor.setInverted(InvertType.OpposeMaster);
		}
		if (mSide == TalonLoc.Cargo_Intake) {
			mPigeon = new PigeonIMU(masterTalon);
			CTRE(mPigeon.configFactoryDefault(200));
			CTRE(mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 3, kLong));
			CTRE(mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 3, kLong));
			CTRE(mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 3, kLong));
		}
		motorSafetyTimer.start(motorTimer);
	}

	public boolean isEncoderConnected() {
		return masterTalon.getSensorCollection().getPulseWidthRiseToRiseUs() > 100;
	}

	public synchronized void set(ControlMode mode, double value, NeutralMode nMode) {
		set(mode, value, DemandType.ArbitraryFeedForward, 0.0, nMode);
	}

	/**
	 * Primary method for all Talon Control. Only sends commands to the Talon if they are new or if the
	 * motor safety timer expires.
	 *
	 * @param mode Control Mode for Talon (PercentOuput, MotionMagic, Velocity, etc.)
	 * @param value Setpoint (Units based on Control Mode, See {@link Constants}
	 * @param nMode Neutral Mode (Brake/Coast) for Talons/Victors
	 * @param arbFeed Arbitrary feedforward added as a PercentOutput to any closed loop (or open loop)
	 *        mode.
	 */
	public synchronized void set(ControlMode mode, double value, DemandType type, double arbFeed, NeutralMode nMode) {
		if (lastNeutralMode != nMode) {
			lastNeutralMode = nMode;
			masterTalon.setNeutralMode(nMode);
			slaveVictor.setNeutralMode(nMode);
		}
		if (mode != lastControlMode || value != lastOutput || lastDemandType != type || arbFeed != lastArbFeed || motorSafetyTimer.isDone()) {
			masterTalon.set(mode, value, type, arbFeed);
			lastControlMode = mode;
			lastOutput = value;
			lastDemandType = type;
			lastArbFeed = arbFeed;
			motorSafetyTimer.start(motorTimer);
		}
	}

	public synchronized void zeroEncoder() {
		if (mSide == TalonLoc.Left || mSide == TalonLoc.Right) {
			CTRE(masterTalon.setSelectedSensorPosition(0, CONFIG.kPIDPrimary, kShort));
		} else if (mSide == TalonLoc.Cargo_Arm) {
			CTRE(masterTalon.getSensorCollection().syncQuadratureWithPulseWidth(CARGO_ARM.kBookEnd_0, CARGO_ARM.kBookEnd_1,
					CARGO_ARM.kCrossOverZero, CARGO_ARM.kOffset, kShort));
		} else if (mSide == TalonLoc.Hatch_Arm) {
			CTRE(masterTalon.getSensorCollection().syncQuadratureWithPulseWidth(HATCH_ARM.kBookEnd_0, HATCH_ARM.kBookEnd_1,
					HATCH_ARM.kCrossOverZero, HATCH_ARM.kOffset, kShort));
		} else {
			Logger.logErrorWithTrace("Can't Zero Encoder: MkTalon Position - " + mSide.toString());
		}
	}

	/**
	 * Update shuffleboard with latest values
	 */
	public synchronized void updateShuffleboard() {
		mVel.setDouble(getVelocity());
		mPos.setDouble(getPosition());
		mError.setDouble(getError());
		mOutput.setDouble(masterTalon.getMotorOutputPercent());
	}

	/**
	 * @return Velocity from SRX Mag Encoder in Inches or Degrees Per Second
	 */
	public synchronized double getVelocity() {
		switch (mSide) {
			case Cargo_Arm:
			case Hatch_Arm:
				return MkMath.nativeUnitsPer100MstoDegreesPerSec(masterTalon.getSelectedSensorVelocity(CONFIG.kPIDPrimary));
			case Left:
			case Right:
				return MkMath.nativeUnitsPer100MstoInchesPerSec(masterTalon.getSelectedSensorVelocity(CONFIG.kPIDPrimary));
			default:
				Logger.logErrorWithTrace("Talon doesn't have encoder");
				return 0.0;
		}
	}

	/**
	 * @return Position from SRX Mag Encoder in Inches or Degrees
	 */
	public synchronized double getPosition() {
		switch (mSide) {
			case Cargo_Arm:
			case Hatch_Arm:
				return MkMath.nativeUnitsToDegrees(masterTalon.getSelectedSensorPosition(CONFIG.kPIDPrimary));
			case Left:
			case Right:
				return MkMath.nativeUnitsToInches(masterTalon.getSelectedSensorPosition(CONFIG.kPIDPrimary));
			default:
				Logger.logErrorWithTrace("Talon doesn't have encoder");
				return 0.0;
		}
	}

	/**
	 * @return Error from setpoint in Inches/Inches Per Sec/Degrees Note that the method returns the
	 *         deviation from target setpoint unlike the official
	 *         {@link BaseMotorController#getClosedLoopError()} method. This method serves to limit CAN
	 *         usage by using known setpoints to calculate error.
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
			case Left:
			case Right:
				if (lastControlMode == ControlMode.Velocity) {
					return MkMath.nativeUnitsPer100MstoInchesPerSec(lastOutput) - getVelocity();
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

	/**
	 * @return Current through Talon in Amperes
	 */
	public synchronized double getCurrent() {
		return masterTalon.getOutputCurrent();
	}

	public boolean checkDriveDeltas() {
		boolean check = true;
		if (currents.size() > 0) {
			Double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();
			if (!Util.allCloseTo(currents, average, TEST.kDriveCurrentEpsilon)) {
				Logger.logErrorWithTrace("Drive Currents varied!!!!!!!!!!!");
				check = true;
			}
		}

		if (positions.size() > 0) {
			Double average = positions.stream().mapToDouble(val -> val).average().getAsDouble();
			if (!Util.allCloseTo(positions, average, TEST.kDrivePosEpsilon)) {
				Logger.logErrorWithTrace("Drive Positions varied!!!!!!!!");
				check = true;
			}
		}

		if (velocities.size() > 0) {
			Double average = velocities.stream().mapToDouble(val -> val).average().getAsDouble();
			if (!Util.allCloseTo(velocities, average, TEST.kDriveVelEpsilon)) {
				Logger.logErrorWithTrace("Drive Velocities varied!!!!!!!!");
				check = false;
			}
		}
		return check;
	}

	/**
	 * Defines the tests for each mechanism. The current, velocity, and position of each mechanism must
	 * meet a minimum (or maximum) value and for mechanisms with several motors, the delta between these
	 * measurements must be below a certain threshold.
	 *
	 * <p>
	 * The cargo and ground intake move to each available setpoint and should be verified by the test
	 * operator.
	 *
	 * @return Whether the test was successful
	 */
	public boolean checkSystem() {
		boolean check = true;

		switch (mSide) {
			case Left:
			case Right:
				MkTime timer = new MkTime();
				CTRE(masterTalon.configFactoryDefault(kLong));
				CTRE(slaveVictor.configFactoryDefault(kLong));
				if (mSide == TalonLoc.Left) {
					masterTalon.setInverted(DRIVE.kLeftInvert);
					masterTalon.setSensorPhase(DRIVE.kLeftSensorInvert);
					slaveVictor.setInverted(DRIVE.kLeftSensorInvert);
				} else {
					masterTalon.setInverted(DRIVE.KRightInvert);
					masterTalon.setSensorPhase(DRIVE.kRightSensorInvert);
					slaveVictor.setInverted(DRIVE.kLeftSensorInvert);
				}
				zeroEncoder();
				masterTalon.setNeutralMode(NeutralMode.Coast);
				slaveVictor.setNeutralMode(NeutralMode.Coast);
				double mCur, mVel, mPos;

				timer.start(3.0);
				while (!timer.isDone()) {
					masterTalon.set(ControlMode.PercentOutput, 0.0);
					slaveVictor.set(ControlMode.PercentOutput, 1.0);
				}

				mVel = getVelocity();
				mPos = getPosition();

				slaveVictor.set(ControlMode.PercentOutput, 0.0);
				masterTalon.set(ControlMode.PercentOutput, 0.0);

				timer.reset();

				velocities.add(mVel);
				positions.add(mPos);

				if (mPos < TEST.kMinDriveTestPos || mVel < TEST.kMinDriveTestVel) {
					Logger.logErrorWithTrace("FAILED - " + mSide.toString() + " Slave FAILED TO REACH REQUIRED SPEED OR POSITION");
					Logger.logMarker(mSide.toString() + " Slave Test Failed - Vel: " + mVel + " Pos: " + mPos);
					check = false;
				} else {
					Logger.logMarker(mSide.toString() + " Slave - Vel: " + mVel + " Pos: " + mPos);
				}

				Timer.delay(2.0);

				zeroEncoder();

				timer.start(3.0);
				while (!timer.isDone()) {
					masterTalon.set(ControlMode.PercentOutput, 1.0);
					slaveVictor.set(ControlMode.PercentOutput, 0.0);
				}

				mVel = getVelocity();
				mCur = getCurrent();
				mPos = getPosition();

				masterTalon.set(ControlMode.PercentOutput, 0.0);
				slaveVictor.set(ControlMode.PercentOutput, 0.0);

				timer.reset();

				currents.add(mCur);
				velocities.add(mVel);
				positions.add(mPos);

				if (mPos < TEST.kMinDriveTestPos || mVel < TEST.kMinDriveTestVel) {
					Logger.logErrorWithTrace("FAILED - " + mSide.toString() + " Master FAILED TO REACH REQUIRED SPEED OR POSITION");
					Logger.logMarker(mSide.toString() + " Master Test Failed - Vel: " + mVel + " Pos: " + mPos);
					check = false;
				} else {
					Logger.logMarker(mSide.toString() + " Master - Vel: " + mVel + " Pos: " + mPos);
				}
				resetConfig();
				break;
			case Cargo_Arm:
				ArrayList<Double> mvelocities = new ArrayList<>();
				ArrayList<Double> mpositions = new ArrayList<>();
				if (!isEncoderConnected()) {
					Logger.logErrorWithTrace("Arm Encoder Not Connected");
					check = false;
				}
				for (CargoArmState state : CargoArmState.values()) {
					if (state != CargoArmState.ENABLE) {
						CargoArm.getInstance().setArmState(state);
						CargoArm.getInstance().setIntakeRollers(-0.5);
						Timer.delay(2.0);
					}
				}
				double current, vel, pos = 0.0;
				/*
				 * Timer.delay(1.0); CargoArm.getInstance().setArmState(CargoArmState.INTAKE);
				 * CargoArm.getInstance().setIntakeRollers(0.0); Timer.delay(1.0);
				 *
				 * MkTime time = new MkTime();
				 *
				 * time.start(2.0);
				 *
				 * while(!time.isDone()){ masterTalon.set(ControlMode.PercentOutput, 0.0);
				 * slaveVictor.set(ControlMode.PercentOutput, -0.4); }
				 *
				 * double current = getCurrent(); double vel = getVelocity(); double pos = getPosition();
				 *
				 * masterTalon.set(ControlMode.PercentOutput, 0.0); slaveVictor.set(ControlMode.PercentOutput, 0.0);
				 * time.reset();
				 *
				 * mvelocities.add(vel); mpositions.add(pos); if (vel < TEST.kMinCargoArmTestVel || pos <
				 * TEST.kMinCargoArmTestPos) { Logger.logErrorWithTrace("FAILED - " + mSide.toString() +
				 * "Cargo Arm Slave FAILED TO REACH REQUIRED SPEED OR POSITION"); Logger.logMarker(mSide.toString()
				 * + " Slave Test Failed - Vel: " + vel + " Pos: " + pos + " Current: " + current); check = false; }
				 * else { Logger.logMarker(mSide.toString() + " Slave - Vel: " + vel + " Pos: " + pos + " Current: "
				 * + current); }
				 *
				 * Timer.delay(1.0);
				 *
				 * CargoArm.getInstance().setArmState(CargoArmState.INTAKE);
				 * CargoArm.getInstance().setIntakeRollers(0.0);
				 *
				 * Timer.delay(1.0);
				 *
				 * time.start(2.0);
				 *
				 * while(!time.isDone()){ masterTalon.set(ControlMode.PercentOutput, -0.4);
				 * slaveVictor.set(ControlMode.PercentOutput, 0.0); }
				 *
				 * vel = getVelocity(); pos = getPosition();
				 *
				 * masterTalon.set(ControlMode.PercentOutput, 0.0); slaveVictor.set(ControlMode.PercentOutput, 0.0);
				 * time.reset();
				 *
				 * mvelocities.add(vel); mpositions.add(pos); if (vel < TEST.kMinCargoArmTestVel || pos >
				 * TEST.kMinCargoArmTestPos) { Logger.logErrorWithTrace("FAILED - " + mSide.toString() +
				 * "Cargo Arm Slave FAILED TO REACH REQUIRED SPEED OR POSITION"); Logger.logMarker(mSide.toString()
				 * + " Master Test Failed - Vel: " + vel + " Pos: " + pos); check = false; } else {
				 * Logger.logMarker(mSide.toString() + " Master - Vel: " + vel + " Pos: " + pos); }
				 *
				 * if (mpositions.size() > 0) { Double average = mpositions.stream().mapToDouble(val ->
				 * val).average().getAsDouble(); if (!Util.allCloseTo(mpositions, average,
				 * TEST.kCargoArmPosEpsilon)) { Logger.logErrorWithTrace(mSide.toString() +
				 * " Positions varied!!!!!!!!"); check = false; } }
				 *
				 * if (mvelocities.size() > 0) { Double average = mvelocities.stream().mapToDouble(val ->
				 * val).average().getAsDouble(); if (!Util.allCloseTo(mvelocities, average,
				 * TEST.kCargoArmVelEpsilon)) { Logger.logErrorWithTrace(mSide.toString() +
				 * " Velocities varied!!!!!!!!"); check = false; } }
				 */
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
				HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.STOWED);
				/*
				 * Timer.delay(1.0); masterTalon.set(ControlMode.PercentOutput, -0.3); Timer.delay(1.0); current =
				 * getCurrent(); vel = getVelocity(); pos = getPosition(); if (vel < TEST.kHatchArmVel || current >
				 * TEST.kHatchArmCurrent || pos < TEST.kHatchArmPos) { Logger.logErrorWithTrace("FAILED - " +
				 * mSide.toString() + "Hatch FAILED TO REACH REQUIRED SPEED OR POSITION");
				 * Logger.logMarker(mSide.toString() + " Test Failed - Vel: " + vel + " Pos: " + pos + " Current: "
				 * + current); check = false; } else { Logger.logMarker(mSide.toString() + " - Vel: " + vel +
				 * " Pos: " + pos + " Current: " + current); }
				 */
				break;
			case Cargo_Intake:
				MkTime newTime = new MkTime();
				CargoArm.getInstance().setArmState(CargoArmState.REVERSE_CARGOSHIP);
				Timer.delay(3.0);
				CargoArm.getInstance().setOpenLoop(0.0);
				newTime.start(15.0);
				while (!slaveTalon.getSensorCollection().isRevLimitSwitchClosed()) {
					if (newTime.isDone()) {
						Logger.logErrorWithTrace("Did not detect reverse cargo limit switch");
						check = false;
						break;
					}
				}
				newTime.reset();
				if (slaveTalon.getSensorCollection().isRevLimitSwitchClosed()) {
					Logger.logMarker("Cargo Reverse Limit Triggered");
				}

				Timer.delay(3.0);
				HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.GROUND_INTAKE);
				Timer.delay(1.0);
				HatchArm.getInstance().setOpenLoop(0.0);
				newTime.start(15.0);
				while (!HatchArm.getInstance().isKetteringReverseTriggered()) {
					if (newTime.isDone()) {
						Logger.logErrorWithTrace("Did not detect reverse kettering switch");
						check = false;
						break;
					}
				}
				newTime.reset();
				if (HatchArm.getInstance().isKetteringReverseTriggered()) {
					Logger.logMarker("Kettering Reverse Limit Triggered");
				}

				newTime.start(10.0);
				while (!CargoArm.getInstance().spearLimit()) {
					if (newTime.isDone()) {
						Logger.logErrorWithTrace("Did not detect spear switch (Fwd)");
						check = false;
						break;
					}
				}
				newTime.reset();
				if (HatchArm.getInstance().isKetteringReverseTriggered()) {
					Logger.logMarker("Spear Limit Triggered (Fwd)");
					check = false;
				}
				break;
			default:
				Logger.logError("Can't Check System!!!");
				break;
		}

		resetConfig();
		return check;
	}

	public void checkForErrorInit() {
		masterTalon.clearStickyFaults();
		Faults masterFaults = new Faults();
		CTRE(masterTalon.getFaults(masterFaults));
		if (masterFaults.hasAnyFault()) {
			if (mSide == TalonLoc.Cargo_Arm || mSide == TalonLoc.Hatch_Arm) {
				zeroEncoder();
			}
			Logger.logMarker(masterFaults.toString());
		}
		// TODO Fix
		if (mSide == TalonLoc.Cargo_Intake || mSide == TalonLoc.Hatch_Arm) {
			slaveTalon.clearStickyFaults();
			Faults slaveTalonFaults = new Faults();
			CTRE(slaveTalon.getFaults(slaveTalonFaults));
			if (slaveTalonFaults.hasAnyFault()) {
				Logger.logMarker(slaveTalonFaults.toString());
			}
		} else {
			slaveVictor.clearStickyFaults();
			Faults slaveVictorFaults = new Faults();
			CTRE(slaveVictor.getFaults(slaveVictorFaults));
			if (slaveVictorFaults.hasAnyFault()) {
				Logger.logMarker(slaveVictorFaults.toString());
			}
		}
	}

	public void checkForReset() {
		StickyFaults masterFaults = new StickyFaults();
		CTRE(masterTalon.getStickyFaults(masterFaults));
		if (masterFaults.hasAnyFault() || masterTalon.hasResetOccurred()) {
			if (mSide == TalonLoc.Cargo_Arm || mSide == TalonLoc.Hatch_Arm) {
				zeroEncoder();
			}
			Logger.logMarker(masterFaults.toString());
		}
	}

	/**
	 * Logs and sends error to DS if any Phoenix Config method returns an Error Code that is not 'OK'
	 */
	private void CTRE(ErrorCode errorCode) {
		if (errorCode != ErrorCode.OK) {
			Logger.logErrorWithTrace(errorCode.toString());
		}
	}

	@Override
	public String toString() {
		return "Output: " + masterTalon.getMotorOutputPercent() + " Current: " + masterTalon.getOutputCurrent()
				+ (mSide != TalonLoc.Cargo_Intake ? " Pos: " + getPosition() + " Vel: " + getVelocity() : " No Encoder");
	}

	/**
	 * Left Drive and Right Drive house the Talon, Victor, and SRX Mag encoder for each side of the
	 * drivetrain. The hatch Arm houses the ground hatch intake Talon, SRX Mag Encoder, and an unused
	 * Talon with a Breakout board with two limit switches. One limit switch is placed at the reverse
	 * hardstop for the ground intake arm, and the second is placed on the pneumatic spear arm to detect
	 * when the main arm is inside the target. The Cargo Arm houses the Talon, Victor, and SRX Mag
	 * encoder for the main cargo arm.
	 */
	public enum TalonLoc {
		Left, Right, Hatch_Arm, Cargo_Arm, Cargo_Intake
	}
}
