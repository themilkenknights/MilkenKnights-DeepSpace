package frc.robot;

import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.InterpolatingTreeMap;

/**
 * Unless otherwise noted by raw/native/RPM, all position unites are in inches and degrees and all velocity units are in inches per second and degrees
 * per second. ID typically notes a CAN ID All PID Constants are in Native Units
 */
public final class Constants {


  public static class GENERAL {

    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 0;
    public static final int kMediumTimeoutMs = 10;
    public static final int kLongCANTimeoutMs = 100; //use for constructors
    public static final double kLoopDt = 0.01;
    public static final double PI = 3.14159265359;
    public static final double kTicksPerRev = 4096.0;
  }


  public static class CAN {

    public static final int kPneumaticsControlModuleID = 0;
    public static final int kDriveLeftMasterID = 4;
    public static final int kDriveLeftSlaveID = 3;
    public static final int kDriveRightMasterID = 7;
    public static final int kDriveRightSlaveID = 6;
    public static final int kPowerDistributionPanelID = 5;
    public static final int kGroundHatchIntakeMotor = 6;
    public static final int kMasterCargoArmMotorID = 7;
    public static final int kSlaveCargoArmMotorID = 8;
    public static final int kLeftCargoIntakeMotorID = 9;
    public static final int kRightCargoIntakeMotorID = 10;
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
    // Tuned dynamics
    public static final double kRobotLinearInertia = 26.30; //Kg
    public static final double kRobotAngularInertia = 4.4;  // Kg m^2 TODO tune
    public static final double kRobotAngularDrag = 6.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 1.07832;  // V
    public static final double kDriveKv = 0.5858;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2
    public static final double kTrackScrubFactor = 1.0;  // TODO tune
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


  }

  public static class CARGO_ARM {

    public static final boolean ARM_SENSOR_PHASE = false;
    public static final boolean ARM_MASTER_DIRECTION = false;
    public static final boolean ARM_SLAVE_DIRECTION = true;
    public static final boolean LEFT_INTAKE_DIRECTION = true;
    public static final boolean RIGHT_INTAKE_DIRECTION = false;

    public static final double RPM_MAX = 35.6; //RPM Max of Arm
    public static final double GEAR_RATIO = 1;//22.0/ 336.0; //Gear Ratio between en;coder and arm - Used to calulate arm position in degrees
    public static final double MAX_RAW_VEL =
        ((RPM_MAX / 60.0 / 10.0) * 4096.0) / GEAR_RATIO; // Degrees per second
    public static final double ARM_P = 22 * ((0.1 * 1023.0) / (1600)); //7.5 deg or 1390 units
    public static final double ARM_I = 0;
    public static final double ARM_D = ARM_P * 52;
    public static final double ARM_F = (1023.0 / MAX_RAW_VEL);

    public static final double ARM_FORWARD_LIMIT = 250;
    public static final double ARM_REVERSE_LIMIT = 0;
    public static final double MOTION_MAGIC_CRUISE_VEL = MAX_RAW_VEL;
    public static final double MOTION_MAGIC_ACCEL = MAX_RAW_VEL * 10;
    public static final double SLOW_INTAKE_HOLD_SPEED = 0.1;
    public static final double MAX_SAFE_CURRENT = 80;

    public static final double INTAKE_IN_ROLLER_SPEED = 0.95; //Intake Roller speed, reverse if it is the wrong direction
    public static final double INTAKE_OUT_ROLLER_SPEED = -0.40;
    public static final double INTAKE_OUT_FAST_ROLLER_SPEED = -0.90;
    //Comp
    public static final int kBookEnd_0 = 827;
    public static final int kBookEnd_1 = 3790;

    //Practice
    //public static final int kBookEnd_0 = 2178;
    //public static final int kBookEnd_1 = 5129;
    public static final double ARM_OFFSET = -127.3;
    public static final double FEED_CONSTANT = 0.15;
  }


  public static class HATCH_ARM {

    public static final int kHatchArmChannel = 1;
    public static final boolean kHatchArmPlaceState = true;

  }

  public static class LOG {

    public static final boolean kDriveCSVLogging = false;
  }


  public static class VISION {

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> visionDistMap = new InterpolatingTreeMap<>();

    static {
      VISION.visionDistMap.put(new InterpolatingDouble(38440.0), new InterpolatingDouble(12.75));
      VISION.visionDistMap.put(new InterpolatingDouble(1392.0), new InterpolatingDouble(23.0));
      VISION.visionDistMap.put(new InterpolatingDouble(7208.0), new InterpolatingDouble(33.0));
      VISION.visionDistMap.put(new InterpolatingDouble(3610.0), new InterpolatingDouble(46.75));
      VISION.visionDistMap.put(new InterpolatingDouble(1742.0), new InterpolatingDouble(66.0));
      VISION.visionDistMap.put(new InterpolatingDouble(782.0), new InterpolatingDouble(96.0));
    }
  }
}
