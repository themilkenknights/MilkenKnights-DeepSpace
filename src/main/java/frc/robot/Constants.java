package frc.robot;

import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.InterpolatingTreeMap;

/**
 * Unless otherwise noted by raw/native/RPM, all position unites are in inches and degrees and all velocity units are in inches per second and
 * degrees per second.
 * ID typically notes a CAN ID
 * All PID Constants are in Native Units
 */
public final class Constants {
		public static final int kSlotIdx = 0;
		public static final int kPIDLoopIdx = 0;
		public static final int kTimeoutMs = 0;
		public static final double kLooperDt = 0.005;
		public static final double PI = 3.14159265359;
		public static final double kTicksPerRev = 4096.0;
		public static final double kDriveWheelTrackWidthInches = 33.75;
		public static final double kTrackScrubFactor = 1.0;  // Tune me!
		// Tuned dynamics
		public static final double kRobotLinearInertia = 26.30; //Kg
		public static final double kRobotAngularInertia = 4.4;  // Kg m^2 TODO tune
		public static final double kRobotAngularDrag = 6.0;  // N*m / (rad/sec) TODO tune
		public static final double kDriveVIntercept = 1.07832;  // V
		public static final double kDriveKv = 0.5858;  // V per rad/s
		public static final double kDriveKa = 0.012;  // V per rad/s^2
		public static final double kPathKX = 4.0;  // units/s per unit of error
		public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
		public static final double kPathMinLookaheadDistance = 24.0;  // inches
		public static final int kLeftMasterID = 10;
		public static final int kLeftSlaveID = 8;
		public static final int kRightMasterID = 5;
		public static final int kRightSlaveID = 3;
		public static final boolean kLeftMasterInvert = false;
		public static final boolean kLeftSlaveInvert = false;
		public static final boolean KRightMasterInvert = true;
		public static final boolean kRightSlaveInvert = true;
		public static final boolean kLeftSensorInvert = true;
		public static final boolean kRightSensorInvert = true;
		public static final double kWheelDiameter = 6.0;
		public static final double kDriveWheelRadiusInches = kWheelDiameter / 2.0;
		public static final double kCircumference = kWheelDiameter * PI;
		public static final double kTurnInPlaceCircumference = 104.1;
		public static final double kLeftMaxRPM = 510.0;
		public static final double kRightMaxRPM = 510.0;
		public static final double kLeftDriveF = (1023.0 / ((kLeftMaxRPM / 60.0 / 10.0) * 4096.0));
		public static final double kRightDriveF = (1023.0 / ((kRightMaxRPM / 60.0 / 10.0) * 4096.0));
		public static final double kMaxVel = (kLeftMaxRPM / 60) * (kCircumference);
		public static final double kMotionMagicCruiseNativeVel = (((kMaxVel / 10.0) / kCircumference) * kTicksPerRev) * 0.5;
		public static final double kMotionMagicNativeAccel = kMotionMagicCruiseNativeVel * 0.5;
		public static final double kDriveP = 7 * (0.1 * 1023.0) / (700); // 300
		public static final double kDriveI = 0;
		public static final double kDriveD = 3 * kDriveP;
		public static final double kMinTestPos = 200;
		public static final double kMinTestVel = 140;
		public static final double kThrottleDeadband = 0.0;
		public static final double kWheelDeadband = 0.0045;
		public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> visionDistMap = new InterpolatingTreeMap<>();

		static {
				visionDistMap.put(new InterpolatingDouble(38440.0), new InterpolatingDouble(12.75));
				visionDistMap.put(new InterpolatingDouble(1392.0), new InterpolatingDouble(23.0));
				visionDistMap.put(new InterpolatingDouble(7208.0), new InterpolatingDouble(33.0));
				visionDistMap.put(new InterpolatingDouble(3610.0), new InterpolatingDouble(46.75));
				visionDistMap.put(new InterpolatingDouble(1742.0), new InterpolatingDouble(66.0));
				visionDistMap.put(new InterpolatingDouble(782.0), new InterpolatingDouble(96.0));
		}
}
