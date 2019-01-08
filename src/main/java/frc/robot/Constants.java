package frc.robot;

import frc.robot.util.math.InterpolatingDouble;
import frc.robot.util.math.InterpolatingTreeMap;

/**
 * UNLESS OTHERWISE NOTED BY RAW/NATIVE/RPM, ALL POSITION UNITS ARE IN INCHES and DEGREES ALL VELOCITY UNITS ARE IN INCHES PER SECOND and
 * DEGREES PER SECOND DIST DENOTES POSITION AND ANG DENOTES ANGLE ID TYPICALLY DENOTES A CAN ID ALL PID CONSTANTS SENT TO THE TALON ARE IN
 * NATIVE UNITS
 */
public final class Constants {

	public static final int kSlotIdx = 0;
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 0;
	public static final double kLooperDt = 0.005;
	public static final double PI = 3.14159265359;
	public static final double CODES_PER_REV = 4096.0;
	public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> visionDistMap = new InterpolatingTreeMap<>();

	static {
		visionDistMap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(42.5));
		visionDistMap.put(new InterpolatingDouble(70.0), new InterpolatingDouble(44.5));
		visionDistMap.put(new InterpolatingDouble(75.0), new InterpolatingDouble(46.8));
		visionDistMap.put(new InterpolatingDouble(80.0), new InterpolatingDouble(48.0));
		visionDistMap.put(new InterpolatingDouble(85.0), new InterpolatingDouble(49.0));
	}

	public static class DRIVE {

		public static final int LEFT_MASTER_ID = 10;
		public static final int LEFT_SLAVE_ID = 8;
		public static final int RIGHT_MASTER_ID = 5;
		public static final int RIGHT_SLAVE_ID = 3;

		public static final boolean LEFT_MASTER_INVERT = false;
		public static final boolean LEFT_SLAVE_INVERT = false;
		public static final boolean RIGHT_MASTER_INVERT = true;
		public static final boolean RIGHT_SLAVE_INVERT = true;

		public static final boolean LEFT_INVERT_SENSOR = true;
		public static final boolean RIGHT_INVERT_SENSOR = true;

		public static final double WHEEL_DIAMETER = 5.95;
		public static final double CIRCUMFERENCE = WHEEL_DIAMETER * PI;
		public static final double TURN_IN_PLACE_CIRCUMFERENCE = 104.1;

		public static final double LEFT_RPM_MAX = 510.0;
		public static final double RIGHT_RPM_MAX = 510.0;

		public static final double MAX_VEL = (LEFT_RPM_MAX / 60) * (CIRCUMFERENCE); // Max Speed in In/s
		public static final double DRIVE_P = 7 * (0.1 * 1023.0) / (700); // 300
		public static final double DRIVE_I = 0;
		public static final double DRIVE_D = 3 * DRIVE_P;
		public static final double LEFT_DRIVE_F = (1023.0 / ((LEFT_RPM_MAX / 60.0 / 10.0) * 4096.0));
		public static final double RIGHT_DRIVE_F = (1023.0 / ((RIGHT_RPM_MAX / 60.0 / 10.0) * 4096.0));
		public static final double MIN_TEST_POS = 200;
		public static final double MIN_TEST_VEL = 140;

		public static final double mPangFollower = -0.075;
		public static final double MOTION_MAGIC_CRUISE_VEL = (((MAX_VEL / 10.0) / CIRCUMFERENCE) * CODES_PER_REV) * 0.5;
		public static final double MOTION_MAGIC_ACCEL = MOTION_MAGIC_CRUISE_VEL * 1;
		public static final double TELEOP_DRIVE_P = 2 * (0.1 * 1023.0) / (700);
		public static final double TELEOP_DRIVE_I = 0;
		public static final double TELEOP_DRIVE_D = 1 * DRIVE_P;
	}

	public static class INPUT {

		public static final double OPERATOR_DEADBAND = 0.1;
		public static final double kThrottleDeadband = 0.0;
		public static final double kWheelDeadband = 0.0045;
	}

}
