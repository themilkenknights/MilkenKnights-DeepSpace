package frc.robot;

import frc.robot.util.math.*;

/**
 * UNLESS OTHERWISE NOTED BY RAW/NATIVE/RPM, ALL POSITION UNITS ARE IN INCHES
 * and DEGREES ALL VELOCITY UNITS ARE IN INCHES PER SECOND and DEGREES PER
 * SECOND DIST DENOTES POSITION AND ANG DENOTES ANGLE ID TYPICALLY DENOTES A CAN
 * ID ALL PID CONSTANTS SENT TO THE TALON ARE IN NATIVE UNITS
 */
public final class Constants {

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 0;
    public static final double kLooperDt = 0.005;
    public static final double PI = 3.14159265359;
    public static final double CODES_PER_REV = 4096.0;

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
        public static final double PATH_WHEELBASE = 33.75;

        public static final double PATH_DIST_TOL = 0.25;
        public static final double PATH_ANGLE_TOL = 0.25;

        public static final double DRIVE_FOLLOWER_P = 1;
        public static final double DRIVE_FOLLOWER_A = 0.00125;
        public static final double DRIVE_FOLLOWER_ANG = -1.25;

        public static final double LEFT_RPM_MAX = 490.0;// 488.0; //Observed Max Speed for Drivetrain in RPM
        public static final double RIGHT_RPM_MAX = 530.0; // 502//Observed Max Speed for Drivetrain in RPM

        public static final double MAX_VEL = (LEFT_RPM_MAX / 60) * (CIRCUMFERENCE); // Max Speed in Inches per second
        public static final double DRIVE_P = 7 * (0.1 * 1023.0) / (700); // 300
        public static final double DRIVE_I = 0;
        public static final double DRIVE_D = 3 * DRIVE_P;
        public static final double LEFT_DRIVE_F = (1023.0 / ((LEFT_RPM_MAX / 60.0 / 10.0) * 4096.0)); // Feedforwrd Term
                                                                                                      // for Drivetrain
                                                                                                      // using MAX Motor
                                                                                                      // Units / Max
                                                                                                      // Speed in Native
                                                                                                      // Units Per 100ms
        public static final double RIGHT_DRIVE_F = (1023.0 / ((RIGHT_RPM_MAX / 60.0 / 10.0) * 4096.0)); // Feedforwrd
                                                                                                        // Term for
                                                                                                        // Drivetrain
                                                                                                        // using MAX
                                                                                                        // Motor Units /
                                                                                                        // Max Speed in
                                                                                                        // Native Units
                                                                                                        // Per 100ms
        public static final double MIN_TEST_POS = 200;
        public static final double MIN_TEST_VEL = 140;

        public static final double mPangFollower = -0.075;
        public static final double MOTION_MAGIC_CRUISE_VEL = ((MAX_VEL / 10.0) / CIRCUMFERENCE) * CODES_PER_REV;
        public static final double MOTION_MAGIC_ACCEL = MOTION_MAGIC_CRUISE_VEL * 5;
        public static final double TELEOP_DRIVE_P = 2 * (0.1 * 1023.0) / (700);
        public static final double TELEOP_DRIVE_I = 0;
        public static final double TELEOP_DRIVE_D = 1 * DRIVE_P;
    }

    public static class LOGGING {

        public static final String DRIVE_LOG_PATH = "DRIVE-LOGS";
        public static final String ARM_LOG_PATH = "ARM-LOGS";
    }

    public static class INPUT {

        public static final double OPERATOR_DEADBAND = 0.1;
        public static final double kThrottleDeadband = 0.0;
        public static final double kWheelDeadband = 0.0045;
    }

    public static class AUTO {

        public static final String pathPath = "/home/lvuser/paths/";
        public static final String[] autoNames = { "CS-1", "CS-2", "CS-3", "CS-4", "CS-5", "CS-6", "CS-7", "CS-8",
                "CS-9", "FS-1", "DriveStraight" };
    }

    public static class SUPERSTRUCTURE {

        public static final int CANIFIER_ID = 11;
        public static double CONNECTION_TIMEOUT = 1.0;
    }

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> visionDistMap = new InterpolatingTreeMap<>();

    static {
        visionDistMap.put(new InterpolatingDouble(60.0), new InterpolatingDouble(42.5));
        visionDistMap.put(new InterpolatingDouble(70.0), new InterpolatingDouble(44.5));
        visionDistMap.put(new InterpolatingDouble(75.0), new InterpolatingDouble(46.8));
        visionDistMap.put(new InterpolatingDouble(80.0), new InterpolatingDouble(48.0));
        visionDistMap.put(new InterpolatingDouble(85.0), new InterpolatingDouble(49.0));
        visionDistMap.put(new InterpolatingDouble(90.0), new InterpolatingDouble(50.5));
        visionDistMap.put(new InterpolatingDouble(100.0), new InterpolatingDouble(52.5));
        visionDistMap.put(new InterpolatingDouble(110.0), new InterpolatingDouble(54.5));
        visionDistMap.put(new InterpolatingDouble(120.0), new InterpolatingDouble(56.5));
        visionDistMap.put(new InterpolatingDouble(130.0), new InterpolatingDouble(57.8));
        visionDistMap.put(new InterpolatingDouble(140.0), new InterpolatingDouble(59.0));
        visionDistMap.put(new InterpolatingDouble(150.0), new InterpolatingDouble(59.8));
        visionDistMap.put(new InterpolatingDouble(160.0), new InterpolatingDouble(60.5));
        visionDistMap.put(new InterpolatingDouble(170.0), new InterpolatingDouble(61.0));
        visionDistMap.put(new InterpolatingDouble(180.0), new InterpolatingDouble(61.5));
    }

}
