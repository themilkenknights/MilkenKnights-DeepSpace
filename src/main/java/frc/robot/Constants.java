package frc.robot;

import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.InterpolatingTreeMap;
import frc.robot.lib.util.Units;

/**
 * Unless otherwise noted by raw/native/RPM, all position unites are in inches and degrees
 * and all velocity units are in inches per second and degrees per second.
 * 'ID' typically notes a CAN ID All PID Constants are in Native Units.
 * The front of the robot is at the Hatch Mechanism/Battery.
 * The exception is for the Cargo Mechanism where Left/Right for this mechanism are flipped.
 *
 * The zero position for the arms are at the hardstops inside the robot perimeter.
 *
 * Positive voltages/sensor measurements for the arms should correspond to rotating toward the ground.
 * Positive Voltages to the drive motors should always move the robot forward.
 *
 * +X is Hatch Arm/Battery Forward
 * -X is Hatch Arm/Battery Forward
 * +Y is Left (Hatch Arm/Battery Forward)
 * -Y is Right (Hatch Arm/Battery Forward)
 */
public final class Constants {

	public static final boolean kIsPracticeBot = true;

	public static class GENERAL {

		public static final int kPIDLoopIdx = 0;
		public static final int kMediumTimeoutMs = 0;
		public static final int kLongCANTimeoutMs = 100; //Use for constructors, not while enabled
		public static final double PI = 3.14159265359;
		public static final double kTicksPerRev = 4096.0;
		public static final double kMaxNominalOutput = 1.0;
		public static final double kLoopDt = 0.02; //TODO Find optimal refresh rate
		public static final double kLooperDt = 0.01;
		public static final double kSlowLooperDt = 0.01;
		public static final double kLimelightLoopPeriod = 0.05;
		public static final double kPixyLoopPeriod = 0.05;
		public static final double kTelemetryDt = 0.2;
	}

	/**
	 * Every CAN ID should be listed here. Note that the Talons & Victors are physical numerical order ony on the Comp Bot.
	 * Every CAN ID corresponds to the same outputs on the practice bot but the order is different.
	 */
	public static class CAN {

		public static final int kPneumaticsControlModuleID = 0;
		public static final int kPowerDistributionPanelID = 11;

		public static final int kDriveLeftMasterTalonID = 5;
		public static final int kDriveLeftSlaveVictorID = 4;

		public static final int kDriveRightMasterTalonID = 10;
		public static final int kDriveRightSlaveVictorID = 9;

		public static final int kGroundHatchArmTalonID = 8;
		public static final int kHatchLimitSwitchTalonID = 3;

		public static final int kMasterCargoArmTalonID = 7;
		public static final int kSlaveCargoArmVictorID = 2;

		public static final int kLeftCargoIntakeTalonID = 1;
		public static final int kRightCargoIntakeVictorID = 6;

	}

	public static class DRIVE {

		//Invert
		public static final boolean kLeftMasterInvert = false;
		public static final boolean kLeftSlaveInvert = false;

		public static final boolean KRightMasterInvert = true;
		public static final boolean kRightSlaveInvert = true;

		public static final boolean kLeftSensorInvert = false;
		public static final boolean kRightSensorInvert = false;

		//Measured params
		public static final double kEffectiveDriveWheelTrackWidthInches = 33.75; //Effective Wheelbase
		public static final double kDriveWheelTrackWidthInches = 26.0;
		public static final double kDriveWheelTrackRadiusMeters = Units.inches_to_meters(kDriveWheelTrackWidthInches / 2.0);
		public static final double kWheelDiameter = 6.0;
		public static final double kCircumference = kWheelDiameter * GENERAL.PI;
		public static final double kDriveWheelRadiusInches = kWheelDiameter / 2.0;

		//Tuned dynamics
		//TODO Tune All Drive Params on Carpet
		public static final double kRobotLinearInertia = 45.30; //Kg
		public static final double kRobotAngularDrag = 6.0;  // N*m / (rad/sec)
		public static final double kDriveVIntercept = 1.07832;  // V
		public static final double kDriveKv = 0.5858;  // V per rad/s2
		public static final double kDriveKa = 0.012;  // V per rad/s^
		public static final double kDriveAngularKa = 0.0169;  // V per rad/s^ (found by turn in place)
		public static final double kRobotAngularInertia = (kDriveWheelTrackRadiusMeters * kDriveKa * kRobotLinearInertia) / (kDriveAngularKa);  // Kg m^2

		//TODO Turn In Place Scrub Tuning
		public static final double kTrackScrubFactor = 0.95;

		//Pure Pursuit Params
		public static final double kPathKX = 4.0;  // units/s per unit of error
		public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
		public static final double kPathMinLookaheadDistance = 24.0;  // inches

		//Talon PID Constants
		public static final double kMaxVel = 160.221;
		public static final double kMaxNativeVel = 3481.6;
		public static final double kMotionMagicCruiseNativeVel = kMaxNativeVel * 0.5;
		public static final double kMotionMagicNativeAccel = kMotionMagicCruiseNativeVel * 0.5;
		public static final double kLeftDriveF = 1023.0 / kMaxNativeVel;
		public static final double kRightDriveF = 1023.0 / kMaxNativeVel;
		public static final double kDriveP = 7 * (0.1 * 1023.0) / (700); // 300
		public static final double kDriveD = 3 * kDriveP;
		public static final double kDriveI = 0;

		//Other
		public static final double kTurnInPlaceCircumference = 104.1;

		//Vision Tuning
		public static final double kVisionTurnP = 0.03;
		public static final double kVisionDriverTurnP = -0.03;

		//Turn In Place
		public static final double kGoalPosTolerance = 0.75; // degrees
		public static final double kGoalVelTolerance = 5.0; // inches per second
	}


	public static class INPUT {

		public static final double kThrottleDeadband = 0.0;
		public static final double kWheelDeadband = 0.003;
		public static final double kOperatorDeadband = 0.01;

	}


	public static class TEST {

		public static final double kMinDriveTestPos = 200;
		public static final double kMinDriveTestVel = 140;
		public static final double kMinDriveTestCurrent = 5;
		public static final double kDriveCurrentEpsilon = 2.0;
		public static final double kDriveVelEpsilon = 2.0;
		public static final double kDrivePosEpsilon = 2.0;

		public static final double kMinCargoArmTestVel = 140;
		public static final double kMinCargoArmTestPos = 140;
		public static final double kMinCargoArmTestCurrent = 5;
		public static final double kCargoArmCurrentEpsilon = 2.0;
		public static final double kCargoArmVelEpsilon = 2.0;
		public static final double kCargoArmPosEpsilon = 2.0;

		public static final double kHatchArmVel = 140;
		public static final double kHatchArmCurrent = 140;
		public static final double kHatchArmPos = 140;

	}

	public static class CARGO_ARM {

		public static final boolean ARM_SENSOR_PHASE = true;
		public static final boolean ARM_MASTER_DIRECTION = false;
		public static final boolean LEFT_INTAKE_DIRECTION = true;
		public static final boolean RIGHT_INTAKE_DIRECTION = false;

		public static final double MAX_RAW_VEL = 243.029333333;
		public static final double ARM_P = 39.0 * ((0.1 * 1023.0) / (1600)); //7.5 deg or 1390 units
		public static final double ARM_I = ARM_P / 300;
		public static final double ARM_D = ARM_P * 27;
		public static final double ARM_F = (1023.0 / MAX_RAW_VEL);

		public static final double ARM_FORWARD_LIMIT = 190.0;
		public static final double ARM_REVERSE_LIMIT = 0.0;
		public static final double MOTION_MAGIC_CRUISE_VEL = MAX_RAW_VEL;
		public static final double MOTION_MAGIC_ACCEL = MAX_RAW_VEL * 10;
		public static final double MAX_SAFE_CURRENT = 80;

		public static final double INTAKE_IN_ROLLER_SPEED = -0.40;
		public static final double INTAKE_OUT_ROLLER_SPEED = 1.0;

		public static final double kArmOffset = -48.4;
		public static final double kFeedConstant = 0.20;

		public static final int kBookEnd_0 = kIsPracticeBot ? 4192 : -2078;
		public static final int kBookEnd_1 = kIsPracticeBot ? 1993 : 3790;
		public static final boolean kCrossOverZero = !kIsPracticeBot;
		public static final int kOffset = kIsPracticeBot ? -4192 : 0;
	}


	public static class HATCH_ARM {

		public static final boolean kHatchArmPlaceState = true;


		public static final boolean ARM_SENSOR_PHASE = false;
		public static final boolean ARM_MASTER_DIRECTION = true;

		public static final double MAX_RAW_VEL = 3085.0;
		public static final double ARM_P = ((5.0 * 1023.0) / (500)); //1.25
		public static final double ARM_I = ARM_P * 0.0;
		public static final double ARM_D = ARM_P * 50.0;
		public static final double ARM_F = (1023.0 / MAX_RAW_VEL);

		public static final double ARM_FORWARD_LIMIT = 175.0;
		public static final double ARM_REVERSE_LIMIT = 0.0;

		public static final double kMotionMagicCruiseVel = MAX_RAW_VEL * 0.9;
		public static final double kMotionMagicAccel = MAX_RAW_VEL * 2;

		public static final double kMaxSafeCurrent = 150;

		public static final int kBookEnd_0 = kIsPracticeBot ? 5453 : 827;
		public static final int kBookEnd_1 = kIsPracticeBot ? 3340 : 3790;
		public static final boolean kCrossOverZero = kIsPracticeBot ? true : true;
		public static final int kOffset = kIsPracticeBot ? -1294 : 0;
	}

	public static class PNUEMATICS {
		public static final int kHatchArmChannel = 0;
		public static final int kFrontClimbSolenoidChannel = 1;
		public static final int kRearClimbSolenoidChannel = 2;
	}

	public static class SUPERSTRUCTURE {
		public static final boolean kClimbUpState = false;
	}

	public static class LOG {

		public static final boolean kDriveCSVLogging = false;
	}


	public static class VISION {

		public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kAreaToDistVisionMap = new InterpolatingTreeMap<>();

		public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPixyAreaToDistVisionMap = new InterpolatingTreeMap<>();

		static {
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(3783.0), new InterpolatingDouble(46.75));
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(2517.0), new InterpolatingDouble(57.4));
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(1960.0), new InterpolatingDouble(65.5));
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(1782.0), new InterpolatingDouble(68.8));
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(1368.0), new InterpolatingDouble(79.4));


			VISION.kPixyAreaToDistVisionMap.put(new InterpolatingDouble(3783.0), new InterpolatingDouble(46.75));
			VISION.kPixyAreaToDistVisionMap.put(new InterpolatingDouble(2517.0), new InterpolatingDouble(57.4));
			VISION.kPixyAreaToDistVisionMap.put(new InterpolatingDouble(1960.0), new InterpolatingDouble(65.5));
		}

	}


}
