package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.math.MkMath;
import java.util.HashMap;
import java.util.Map;

/**
 * Unless otherwise noted by raw/native/RPM, all position unites are in inches and degrees and all velocity units are in inches per second and degrees
 * per second. 'ID' typically notes a CAN ID All PID Constants are in Native Units. The front of the robot is at the Hatch Mechanism/Battery. The
 * exception is for the Cargo Mechanism where Left/Right for this mechanism are flipped.
 *
 * <p>
 * The zero position for the arms are at the hardstops inside the robot perimeter.
 *
 * <p>
 * Positive voltages/sensor measurements for the arms should correspond to rotating toward the ground. Positive Voltages to the drive motors should
 * always move the robot forward.
 *
 * <p>
 * +X is Hatch Arm/Battery Forward -X is Hatch Arm/Battery Forward +Y is Left (Hatch Arm/Battery Forward) -Y is Right (Hatch Arm/Battery Forward)
 */
public final class Constants {
  public static final boolean kIsPracticeBot = false;

  public static class GENERAL {
    public static final double PI = 3.14159265359;
    public static final double kMaxNominalOutput = 1.0;
    public static final double kTicksPerRev = 4096.0;
    public static final double kMotorSafetyTimer = 0.05;
    public static final double kMainLoopDt = 0.02;
    public static final double kFastLooperDt = 0.005;
    public static final double kThrottleDeadband = 0.0;
    public static final double kWheelDeadband = 0.003;
    public static final double kOperatorDeadband = 0.01;
  }

  /**
   * Every CAN ID should be listed here. Note that the Talons & Victors are physical numerical order ony on the Comp Bot. Every CAN ID corresponds to
   * the same outputs on the practice bot but the order is different.
   */
  public static class CAN {
    public static final int kPneumaticsControlModuleID = 0;
    public static final int kPowerDistributionPanelID = 11;
    // SRX Mag Encoder for Left Drive
    public static final int kDriveLeftMasterTalonID = 10;
    // Empty
    public static final int kDriveLeftSlaveVictorID = 9;
    // SRX Mag Encoder for Right Drive
    public static final int kDriveRightMasterTalonID = 5;
    // Empty
    public static final int kDriveRightSlaveVictorID = 4;
    public static final int kGroundHatchArmTalonID = 8;
    // Kettering Reverse Limit
    public static final int kKetteringReverseLimitSwitchTalonID = 3;
    // SRX Mag Encoder for Cargo Arm
    public static final int kMasterCargoArmTalonID = 7;
    // Empty
    public static final int kSlaveCargoArmVictorID = 2;
    // Used for Cargo Reverse Limit Switch and Spear Limit
    public static final int kLeftCargoIntakeTalonID = 1;
    // Used for Pigeon IMU
    public static final int kRightCargoIntakeTalonID = 6;
  }

  public static class DRIVE {
    // Invert
    public static final boolean kLeftInvert = false;
    public static final boolean KRightInvert = true;
    public static final boolean kLeftSensorInvert = true;
    public static final boolean kRightSensorInvert = true;
    // Measured params
    public static final double kEffectiveDriveWheelTrackWidthInches = 33.75;
    // Effective Wheelbase
    public static final double kDriveWheelTrackWidthInches = 22.97;
    public static final double kWheelDiameter = 5.9575;
    public static final double kCircumference = kWheelDiameter * GENERAL.PI;
    public static final double kTrackScrubFactor = 0.95;
    // Talon PID Constants
    public static final double kMaxVel = 154.62;
    public static final double kMaxNativeVel = 3370.0;
    public static final double kGoalPosTolerance = 1.5;
    // Vision Tuning
    public static final double DRIVE_FOLLOWER_P = 1;
    public static final double DRIVE_FOLLOWER_A = 0.00125;
    public static final double DRIVE_FOLLOWER_ANG = -1.25;
    public static final double PATH_DT = 0.01;

    public static final double kVisionP = 0.00688; //0.0087
    public static final double kVisionI = 0.0;
    public static final double kVisionD = 590.000666; //430
  }

  public static class TEST {
    public static final double kMinDriveTestPos = 420.0;
    public static final double kMinDriveTestVel = 145.0;
    public static final double kDriveCurrentEpsilon = 3.0;
    public static final double kDriveVelEpsilon = 1.5;
    public static final double kDrivePosEpsilon = 3.0;
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
    public static final boolean kCargoArmSensorPhase = true;
    public static final boolean kCargoArmDirection = false;
    public static final boolean kLeftIntakeDirection = true;
    public static final boolean kRightIntakeDirection = false;
    public static final double kMaxRawVel = 243.029333333;
    public static final double kMaxSafeCurrent = 80;
    public static final double kIntakeRollerInSpeed = -0.40;
    public static final double kCargoShipIntakeRollerOut = 0.5;
    public static final double kFrontCargoShipIntakeRollerOut = 0.44;
    public static final double kRocketLevelOneOutSpeed = 0.3;
    public static final double kRocketLevelTwoOutSpeed = 0.70;
    public static final double kDefaultIntakeRollerOutSpeed = 0.3;
    public static final double kArmOffset = -48.4;
    public static final double kFeedConstant = 0.20;
    public static final int kBookEnd_0 = kIsPracticeBot ? 4192 : 3086;
    public static final int kBookEnd_1 = kIsPracticeBot ? 1993 : 919;
    public static final boolean kCrossOverZero = false;
    public static final int kOffset = kIsPracticeBot ? -4192 : -3086;
  }

  public static class MISC {
    public static final boolean kDriveCSVLogging = false;
    public static final boolean kErrorLogging = true;
    public static final int kFirstHatchArmChannel = 0;
    public static final int kFrontClimbSolenoidChannel = 2;
    public static final int kRearClimbSolenoidChannel = 1;
    public static final int kHatchPancakeChannel = 3;
  }

  public static class CONFIG {
    public static final int kPIDPrimary = 0;
    public static final int kPIDAuxilliaryTurn = 1;
    public static final Map<TalonLoc, TalonSRXConfiguration> kConfigs = new HashMap<>();

    static {
      kConfigs.put(TalonLoc.Left, new TalonSRXConfiguration());
      kConfigs.put(TalonLoc.Right, new TalonSRXConfiguration());
      kConfigs.put(TalonLoc.Cargo_Arm, new TalonSRXConfiguration());
      kConfigs.put(TalonLoc.Cargo_Intake, new TalonSRXConfiguration());
      for (TalonLoc loc : kConfigs.keySet()) {
        TalonSRXConfiguration tal = kConfigs.get(loc);
        tal.peakOutputForward = GENERAL.kMaxNominalOutput;
        tal.peakOutputReverse = -GENERAL.kMaxNominalOutput;
        tal.neutralDeadband = 0.0;
        tal.voltageCompSaturation = 12.0;
        tal.voltageMeasurementFilter = 32;
        tal.neutralDeadband = 0.0;
        // Ensure that disconnected limit switches do not cause arm motors to stop
        tal.remoteSensorClosedLoopDisableNeutralOnLOS = true;
        tal.limitSwitchDisableNeutralOnLOS = true;
        tal.softLimitDisableNeutralOnLOS = true;
        tal.motionCurveStrength = 4;
        // Disable all limit switches by default
        tal.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
        tal.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
        tal.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
        tal.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
        if (loc == TalonLoc.Left || loc == TalonLoc.Right) {
          tal.motionCurveStrength = 0;
          tal.velocityMeasurementPeriod = VelocityMeasPeriod.Period_25Ms;
          tal.velocityMeasurementWindow = 16;
          // General Velocity/Motion Magic
          tal.slot0.kP = 7.0 * (0.1 * 1023.0) / (700.0);
          tal.slot0.kD = 3.0 * tal.slot0.kP;
          tal.slot0.kF = 1023.0 / DRIVE.kMaxNativeVel;
          tal.slot0.closedLoopPeakOutput = 1.0;
          tal.slot0.allowableClosedloopError = 100;
          tal.motionCruiseVelocity = (int) (DRIVE.kMaxNativeVel * 0.8);
          tal.motionAcceleration = (int) (tal.motionCruiseVelocity * 0.5);
          tal.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
        } else if (loc == TalonLoc.Cargo_Arm) {
          tal.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
          tal.reverseLimitSwitchSource = LimitSwitchSource.RemoteTalonSRX;
          tal.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
          tal.forwardSoftLimitEnable = true;
          tal.reverseSoftLimitEnable = true;
          tal.clearPositionOnLimitR = false;
        }
        if (loc == TalonLoc.Cargo_Arm) {
          tal.reverseLimitSwitchDeviceID = CAN.kRightCargoIntakeTalonID;
          tal.forwardSoftLimitThreshold = (int) MkMath.degreesToNativeUnits(195);
          tal.reverseSoftLimitThreshold = (int) MkMath.degreesToNativeUnits(14.0);
          tal.motionCruiseVelocity = (int) (CARGO_ARM.kMaxRawVel);
          tal.motionAcceleration = (int) (CARGO_ARM.kMaxRawVel * 10);
          tal.slot0.kP = (39.0 * ((0.1 * 1023.0) / (1600)));
          tal.slot0.kI = tal.slot0.kP / 210;
          tal.slot0.kD = tal.slot0.kP * 40;
          tal.slot0.kF = 1023.0 / CARGO_ARM.kMaxRawVel;
          tal.slot0.integralZone = 200;
          tal.slot0.maxIntegralAccumulator = 300;
          tal.motionCurveStrength = 3;
        }
      }
    }
  }
}
