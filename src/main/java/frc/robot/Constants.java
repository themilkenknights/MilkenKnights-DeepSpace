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
      public static final int kLeftCargoArmMotorID = 7;
      public static final int kRightCargoArmMotorID = 8;

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
  }


  public static class TEST {

    public static final double kMinTestPos = 200;
    public static final double kMinTestVel = 140;
    public static final double kMinTestCurrent = 5;

  }


  public static class HATCH {

    public static final int kHatchArmForwardChannel = 1;
    public static final int kHatchArmReverseChannel = 2;
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
