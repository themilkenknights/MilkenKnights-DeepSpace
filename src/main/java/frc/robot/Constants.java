package frc.robot;

import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.InterpolatingTreeMap;

/**
 * Unless otherwise noted by raw/native/RPM, all position unites are in inches and degrees and all velocity units are in inches per second and degrees per second. ID
 * typically notes a CAN ID All PID Constants are in Native Units. The front of the robot is at the Hatch Mechanism/Battery. The exception is for the Cargo Mechanism.
 * Left/Right for this mechanism are flipped. The zero position for the arms are at the hardstops inside the robot perimeter. Positive voltages/sensor measurements for
 * the arms should correspond to rotating toward the ground. Positive Voltages to the drive motors should always move the robot forward.
 */
public final class Constants {

	public static final boolean kIsPracticeBot = true;

	public static class GENERAL {

		public static final int kPIDLoopIdx = 0;
		public static final int kMediumTimeoutMs = 10;
		public static final int kLongCANTimeoutMs = 100; //use for constructors
		public static final double kLoopDt = 0.01;
		public static final double PI = 3.14159265359;
		public static final double kTicksPerRev = 4096.0;
		public static final double kMaxNominalOutput = 0.3;

	}

	public static class CAN {

		public static final int kPneumaticsControlModuleID = 0;
		public static final int kPowerDistributionPanelID = 11;

		public static final int kDriveLeftMasterTalonID = 10;
		public static final int kDriveLeftSlaveVictorID = 7;

		public static final int kDriveRightMasterTalonID = 5;
		public static final int kDriveRightSlaveVictorID = 2;

		public static final int kGroundHatchArmTalonID = 9;

		public static final int kHatchLimitSwitchTalonID = 4;

		public static final int kRightMasterCargoArmTalonID = 8;
		
		public static final int kLeftSlaveCargoArmVictorID = 1;

		public static final int kLeftCargoIntakeTalonID = 3;

		public static final int kRightCargoIntakeVictorID = 6;
	}

	public static class DRIVE {

		//Invert
		public static final boolean kLeftMasterInvert = false;
		public static final boolean kLeftSlaveInvert = false;
		public static final boolean KRightMasterInvert = true;
		public static final boolean kRightSlaveInvert = true;
		public static final boolean kLeftSensorInvert = true;
		public static final boolean kRightSensorInvert = true;
		//Measured params
		public static final double kDriveWheelTrackWidthInches = 33.75; //Effective Wheelbase
		public static final double kWheelDiameter = 6.0;
		public static final double kCircumference = kWheelDiameter * GENERAL.PI;
		public static final double kDriveWheelRadiusInches = kWheelDiameter / 2.0;

		// Tuned dynamics TODO Tune All Drive Params on Carpet
		public static final double kRobotLinearInertia = 26.30; //Kg
		public static final double kRobotAngularInertia = 4.4;  // Kg m^2
		public static final double kRobotAngularDrag = 6.0;  // N*m / (rad/sec)
		public static final double kDriveVIntercept = 1.07832;  // V
		public static final double kDriveKv = 0.5858;  // V per rad/s
		public static final double kDriveKa = 0.012;  // V per rad/s^2
		public static final double kTrackScrubFactor = 1.0;
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
		public static final double kVisionTurnP = 0.08;
		//Turn In Place
		public static final double kGoalPosTolerance = 0.75; // degrees
		public static final double kGoalVelTolerance = 5.0; // inches per second
	}


	public static class INPUT {

		public static final double kThrottleDeadband = 0.0;
		public static final double kWheelDeadband = 0.0045;
		public static final double kOperatorDeadband = 0.0045;

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
		public static final boolean ARM_MASTER_DIRECTION = true;
		public static final boolean LEFT_INTAKE_DIRECTION = true;
		public static final boolean RIGHT_INTAKE_DIRECTION = false;

		public static final double MAX_RAW_VEL = 243.029333333;
		public static final double ARM_P = 22 * ((0.1 * 1023.0) / (1600)); //7.5 deg or 1390 units
		public static final double ARM_I = 0;
		public static final double ARM_D = ARM_P * 52;
		public static final double ARM_F = (1023.0 / MAX_RAW_VEL);

		public static final double ARM_FORWARD_LIMIT = 100;
		public static final double ARM_REVERSE_LIMIT = 0;
		public static final double MOTION_MAGIC_CRUISE_VEL = MAX_RAW_VEL;
		public static final double MOTION_MAGIC_ACCEL = MAX_RAW_VEL * 10;
		public static final double MAX_SAFE_CURRENT = 80;

		public static final double INTAKE_IN_ROLLER_SPEED = 0.95;
		public static final double INTAKE_OUT_ROLLER_SPEED = -0.40;

		public static final double kArmOffset = -127.3;
		public static final double kFeedConstant = 0.15;

		public static final int kBookEnd_0 = kIsPracticeBot ? 827 : 827;
		public static final int kBookEnd_1 = kIsPracticeBot ? 3790 : 3790;
		public static final boolean kCrossOverZero = kIsPracticeBot ? true : true;
		public static final int kOffset = kIsPracticeBot ? 0 : 0;
	}


	public static class HATCH_ARM {

		public static final int kHatchArmChannel = 0;
		public static final boolean kHatchArmPlaceState = true;


		public static final boolean ARM_SENSOR_PHASE = false;
		public static final boolean ARM_MASTER_DIRECTION = false;

		public static final double MAX_RAW_VEL = 243.029333333;
		public static final double ARM_P = 22 * ((0.1 * 1023.0) / (1600));
		public static final double ARM_I = 0;
		public static final double ARM_D = ARM_P * 52;
		public static final double ARM_F = (1023.0 / MAX_RAW_VEL);

		public static final double ARM_FORWARD_LIMIT = 100;
		public static final double ARM_REVERSE_LIMIT = 0;

		public static final double kMotionMagicCruiseVel = MAX_RAW_VEL;
		public static final double kMotionMagicAccel = MAX_RAW_VEL * 10;

		public static final double kMaxSafeCurrent = 40;

		public static final int kBookEnd_0 = kIsPracticeBot ? 827 : 827;
		public static final int kBookEnd_1 = kIsPracticeBot ? 3790 : 3790;
		public static final boolean kCrossOverZero = kIsPracticeBot ? true : true;
		public static final int kOffset = kIsPracticeBot ? 0 : 0;

	}

	public static class LOG {

		public static final boolean kDriveCSVLogging = false;
	}


	public static class VISION {

		public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kAreaToDistVisionMap = new InterpolatingTreeMap<>();


		static {
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(38440.0), new InterpolatingDouble(12.75));
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(1392.0), new InterpolatingDouble(23.0));
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(7208.0), new InterpolatingDouble(33.0));
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(3610.0), new InterpolatingDouble(46.75));
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(1742.0), new InterpolatingDouble(66.0));
			VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(782.0), new InterpolatingDouble(96.0));

		}
	}


}
