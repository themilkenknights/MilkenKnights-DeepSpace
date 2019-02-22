package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.InterpolatingTreeMap;
import java.util.HashMap;
import java.util.Map;

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
        public static final double kMotorSafetyTimer = 0.05;
        public static final double kMainLoopDt = 0.02;
        public static final double kFastLooperDt = 0.02;
        public static final double kLimelightLoopPeriod = 0.01;
        public static final double kPixyLoopPeriod = 0.01;
    }


    /**
     * Every CAN ID should be listed here. Note that the Talons & Victors are physical numerical order ony on the Comp Bot.
     * Every CAN ID corresponds to the same outputs on the practice bot but the order is different.
     */
    public static class CAN {

        public static final int kPneumaticsControlModuleID = 0;
        public static final int kPowerDistributionPanelID = 11;

        //SRX Mag Encoder for Left Drive
        public static final int kDriveLeftMasterTalonID = 10;

        //Empty
        public static final int kDriveLeftSlaveVictorID = 9;

        //SRX Mag Encoder for Right Drive
        public static final int kDriveRightMasterTalonID = 5;

        //Empty
        public static final int kDriveRightSlaveVictorID = 4;

        public static final int kGroundHatchArmTalonID = 8;

        //Kettering Reverse Limit
        public static final int kHatchReverseLimitSwitchTalonID = 3;

        //SRX Mag Encoder for Cargo Arm
        public static final int kMasterCargoArmTalonID = 7;

        //Empty
        public static final int kSlaveCargoArmVictorID = 2;

        //Used for Cargo Reverse Limit Switch and Spear Limit
        public static final int kLeftCargoIntakeTalonID = 1;

        //Used for Pigeon IMU
        public static final int kRightCargoIntakeTalonID = 6;
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
        public static final double kEffectiveDriveWheelTrackWidthInches = 33.75; //Effective Wheelbase
        public static final double kDriveWheelTrackWidthInches = 22.45;
        public static final double kDriveWheelTrackRadiusMeters = (kDriveWheelTrackWidthInches / 2.0) * 0.0254;
        public static final double kWheelDiameter = 6.0;
        public static final double kCircumference = kWheelDiameter * GENERAL.PI;
        public static final double kDriveWheelRadiusInches = kWheelDiameter / 2.0;

        //Tuned dynamics
        //TODO Tune All Drive Params on Carpet
        public static final double kRobotLinearInertia = 45.94891; //Kg
        public static final double kRobotAngularDrag = 10.0;  // N*m / (rad/sec)
        public static final double kDriveVIntercept = 0.65;  // V
        public static final double kDriveKv = 0.275;  // V per rad/s2
        public static final double kDriveKa = 0.00575;  // V per rad/s^
        public static final double kDriveAngularKa = 0.0065;  // V per rad/s^ (found by turn in place)
        public static final double kRobotAngularInertia = (kDriveWheelTrackRadiusMeters * kDriveKa * kRobotLinearInertia) / (kDriveAngularKa);  // Kg m^2

        //TODO Turn In Place Scrub Tuning
        public static final double kTrackScrubFactor = 0.95;

        //Pure Pursuit Params
        public static final double kPathKX = 0.9;  // units/s per unit of error
        public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
        public static final double kPathMinLookaheadDistance = 24.0;  // inches

        //Talon PID Constants
        public static final double kMaxVel = 154.62;
        public static final double kMaxNativeVel = 3370.0;
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

        public static final double kVisionTurnP = 0.040;
        public static final double kVisionDriverTurnP = -0.03;


        //Turn In Place
        public static final double kGoalPosTolerance = 0.75; // degrees
        public static final double kGoalVelTolerance = 5.0; // inches per second

        public static final byte kNavXUpdateRate = (byte) 200;

        public static final double kPixyKp = 0.045;
    }


    public static class INPUT {

        public static final double kThrottleDeadband = 0.0;
        public static final double kWheelDeadband = 0.003;
        public static final double kOperatorDeadband = 0.01;

    }


    public static class TEST {

        public static final double kMinDriveTestPos = 180;
        public static final double kMinDriveTestVel = 60;
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
        public static final double ARM_REVERSE_LIMIT = -13.0;
        public static final double MOTION_MAGIC_CRUISE_VEL = MAX_RAW_VEL;
        public static final double MOTION_MAGIC_ACCEL = MAX_RAW_VEL * 10;
        public static final double MAX_SAFE_CURRENT = 80;

        public static final double INTAKE_IN_ROLLER_SPEED = -0.40;
        public static final double CARGOSHIP_INTAKE_OUT_ROLLER_SPEED = 0.3;
        public static final double ROCKET_LEVEL_ONE_INTAKE_OUT_ROLLER_SPEED = 0.3;
        public static final double ROCKET_LEVEL_TWO_OUT_ROLLER_SPEED = 0.3;
        public static final double DEFAULT_INTAKE_ROLLER_OUT_SPEED = 0.3;

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
        public static final int kOffset = kIsPracticeBot ? -1336 : 0;
    }


    public static class PNUEMATICS {


        public static final int kHatchArmChannel = 0;
        public static final int kFrontClimbSolenoidChannel = 1;
        public static final int kRearClimbSolenoidChannel = 2;
    }


    public static class SUPERSTRUCTURE {

        public static final boolean kClimbRetractedState = false;
    }


    public static class LOG {

        public static final boolean kDriveCSVLogging = false;
    }


    public static class VISION {

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kAreaToDistVisionMap = new InterpolatingTreeMap<>();

        public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kPixyAreaToDistVisionMap = new InterpolatingTreeMap<>();

        static {
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(12400.0), new InterpolatingDouble(15.9));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(9362.0), new InterpolatingDouble(20.16));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(6958.0), new InterpolatingDouble(26.26));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(5122.0), new InterpolatingDouble(32.45));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(3423.0), new InterpolatingDouble(42.8));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(2155.0), new InterpolatingDouble(57.44));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(1464.0), new InterpolatingDouble(73.35));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(980.0), new InterpolatingDouble(94.09));

            VISION.kPixyAreaToDistVisionMap.put(new InterpolatingDouble(3783.0), new InterpolatingDouble(46.75));
            VISION.kPixyAreaToDistVisionMap.put(new InterpolatingDouble(2517.0), new InterpolatingDouble(57.4));
            VISION.kPixyAreaToDistVisionMap.put(new InterpolatingDouble(1960.0), new InterpolatingDouble(65.5));
        }

    }


    public static class CONFIG {
        public static final Map<TalonLoc, TalonSRXConfiguration> kConfigs = new HashMap<TalonLoc, TalonSRXConfiguration>();

        static {
            kConfigs.put(TalonLoc.Left, new TalonSRXConfiguration());
            kConfigs.put(TalonLoc.Right, new TalonSRXConfiguration());
            kConfigs.put(TalonLoc.Cargo_Arm, new TalonSRXConfiguration());
            kConfigs.put(TalonLoc.Hatch_Arm, new TalonSRXConfiguration());
            kConfigs.put(TalonLoc.Cargo_Intake, new TalonSRXConfiguration());
            for (TalonLoc loc : kConfigs.keySet()) {
                TalonSRXConfiguration tal = kConfigs.get(loc);
                if (loc == TalonLoc.Left || loc == TalonLoc.Right) {
                    tal.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
                    tal.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
                    tal.sum0Term = FeedbackDevice.CTRE_MagEncoder_Relative;
                    tal.sum1Term = FeedbackDevice.RemoteSensor0;
                    tal.diff0Term = FeedbackDevice.RemoteSensor1;
                    tal.diff1Term = FeedbackDevice.PulseWidthEncodedPosition;
                    tal.peakOutputForward = 1.0;
                    tal.peakOutputReverse = -1.0;
                    tal.neutralDeadband = 0.0;
                    tal.voltageCompSaturation = 12.0;
                    tal.voltageMeasurementFilter = 32;
                    tal.velocityMeasurementPeriod = VelocityMeasPeriod.Period_25Ms;
                    tal.velocityMeasurementWindow = 8;
                    tal.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
                    tal.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
                    tal.forwardSoftLimitEnable = false;
                    tal.reverseSoftLimitEnable = false;
                    tal.slot0.kP = 7 * (0.1 * 1023.0) / (700);
                    tal.slot0.kI = 0.0;
                    tal.slot0.kD = 3 * tal.slot0.kP;
                    tal.slot0.kF = 1023.0 / DRIVE.kMaxNativeVel;
                    tal.slot0.integralZone = 100;
                    tal.slot0.allowableClosedloopError = 1;
                    tal.slot0.maxIntegralAccumulator = 0.0;
                    tal.slot0.closedLoopPeakOutput = 1.0;
                    tal.slot0.closedLoopPeriod = 1;
                    tal.slot1.kP = 0.1;
                    tal.slot1.kI = 0.0;
                    tal.slot1.kD = 0.0;
                    tal.slot1.kF = 0.0;
                    tal.slot1.integralZone = 100;
                    tal.slot1.allowableClosedloopError = 200;
                    tal.slot1.maxIntegralAccumulator = 91.000000;
                    tal.slot1.closedLoopPeakOutput = 0.5;
                    tal.slot1.closedLoopPeriod = 1;
                    tal.slot2.kP = 2.0;
                    tal.slot2.kI = 0.0;
                    tal.slot2.kD = 4.0;
                    tal.slot2.kF = 6.323232;
                    tal.slot2.integralZone = 200;
                    tal.slot2.allowableClosedloopError = 343;
                    tal.slot2.maxIntegralAccumulator = 334.000000;
                    tal.slot2.closedLoopPeakOutput = 1.0;
                    tal.slot2.closedLoopPeriod = 1;
                    tal.slot3.kP = 34.000000;
                    tal.slot3.kI = 32.000000;
                    tal.slot3.kD = 436.000000;
                    tal.slot3.kF = 0.343430;
                    tal.slot3.integralZone = 2323;
                    tal.slot3.allowableClosedloopError = 543;
                    tal.slot3.maxIntegralAccumulator = 687.000000;
                    tal.slot3.closedLoopPeakOutput = 0.129032;
                    tal.slot3.closedLoopPeriod = 1;
                    tal.auxPIDPolarity = false;
                    tal.motionCruiseVelocity = (int) (DRIVE.kMaxNativeVel * 0.5);
                    tal.motionAcceleration = (int) (tal.motionCruiseVelocity * 0.5);
                    tal.motionCurveStrength = 6;
                    tal.feedbackNotContinuous = false;
                    tal.remoteSensorClosedLoopDisableNeutralOnLOS = false;
                    tal.limitSwitchDisableNeutralOnLOS = true;
                    tal.softLimitDisableNeutralOnLOS = false;
                } else if (loc == TalonLoc.Cargo_Arm || loc == TalonLoc.Hatch_Arm) {
                    tal.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
                    tal.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
                    tal.reverseLimitSwitchSource = LimitSwitchSource.RemoteTalonSRX;
                    tal.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
                    tal.peakOutputForward = 1.0;
                    tal.peakOutputReverse = -1.0;
                    tal.neutralDeadband = 0.0;
                    tal.voltageCompSaturation = 12.0;
                    tal.voltageMeasurementFilter = 32;
                    tal.velocityMeasurementPeriod = VelocityMeasPeriod.Period_25Ms;
                    tal.velocityMeasurementWindow = 8;
                    tal.forwardSoftLimitEnable = true;
                    tal.reverseSoftLimitEnable = true;
                    tal.feedbackNotContinuous = false;
                    tal.remoteSensorClosedLoopDisableNeutralOnLOS = false;
                    tal.limitSwitchDisableNeutralOnLOS = true;
                    tal.softLimitDisableNeutralOnLOS = false;
                    tal.motionCurveStrength = 6;
                    tal.slot0.closedLoopPeakOutput = 1.0;
                    tal.slot0.closedLoopPeriod = 1;
                } else if (loc == TalonLoc.Cargo_Intake) {
                    tal.peakOutputForward = 1.0;
                    tal.peakOutputReverse = -1.0;
                    tal.neutralDeadband = 0.0;
                    tal.voltageCompSaturation = 12.0;
                    tal.voltageMeasurementFilter = 32;
                }

                if (loc == TalonLoc.Left) {
                    tal.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
                    tal.primaryPID.selectedFeedbackCoefficient = 1.0;
                } else if (loc == TalonLoc.Right) {
                    tal.remoteFilter0.remoteSensorDeviceID = CAN.kDriveLeftMasterTalonID;
                    tal.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonSRX_SelectedSensor;

                    tal.remoteFilter1.remoteSensorDeviceID = CAN.kRightCargoIntakeTalonID;
                    tal.remoteFilter1.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Yaw;

                    tal.sum0Term = FeedbackDevice.RemoteSensor0;
                    tal.sum1Term = FeedbackDevice.CTRE_MagEncoder_Relative;

                    tal.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorSum;
                    tal.primaryPID.selectedFeedbackCoefficient = 0.50;

                    tal.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1;
                    tal.auxiliaryPID.selectedFeedbackCoefficient = 1.0;
                } else if (loc == TalonLoc.Cargo_Arm) {
                    tal.reverseLimitSwitchDeviceID = CAN.kLeftCargoIntakeTalonID;
                    tal.forwardSoftLimitThreshold = 0;
                    tal.reverseSoftLimitThreshold = 0;
                    tal.motionCruiseVelocity = (int) (DRIVE.kMaxNativeVel * 0.5);
                    tal.motionAcceleration = (int) (tal.motionCruiseVelocity * 0.5);
                    tal.slot0.kP = 7 * (0.1 * 1023.0) / (700);
                    tal.slot0.kI = 0.0;
                    tal.slot0.kD = 3 * tal.slot0.kP;
                    tal.slot0.kF = 1023.0 / DRIVE.kMaxNativeVel;
                    tal.slot0.integralZone = 100;
                    tal.slot0.allowableClosedloopError = 1;
                    tal.slot0.maxIntegralAccumulator = 0.0;
                } else if (loc == TalonLoc.Hatch_Arm) {
                    tal.reverseLimitSwitchDeviceID = CAN.kHatchReverseLimitSwitchTalonID;
                    tal.forwardSoftLimitThreshold = 0;
                    tal.reverseSoftLimitThreshold = 0;
                    tal.motionCruiseVelocity = (int) (DRIVE.kMaxNativeVel * 0.5);
                    tal.motionAcceleration = (int) (tal.motionCruiseVelocity * 0.5);
                    tal.slot0.kP = 7 * (0.1 * 1023.0) / (700);
                    tal.slot0.kI = 0.0;
                    tal.slot0.kD = 3 * tal.slot0.kP;
                    tal.slot0.kF = 1023.0 / DRIVE.kMaxNativeVel;
                    tal.slot0.integralZone = 100;
                    tal.slot0.allowableClosedloopError = 1;
                    tal.slot0.maxIntegralAccumulator = 0.0;
                } else if (loc == TalonLoc.Cargo_Intake) {

                }
            }
        }
    }
}
