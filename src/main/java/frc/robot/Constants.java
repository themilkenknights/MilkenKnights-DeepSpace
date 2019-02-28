package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.math.MkMath;
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
        public static final int kShortTimeoutMs = 0;
        public static final int kLongCANTimeoutMs = 50; //Use for constructors, not while enabled
        public static final double PI = 3.14159265359;
        public static final double kMaxNominalOutput = 1.0;
        public static final double kTicksPerRev = 4096.0;
        public static final double kMotorSafetyTimer = 0.05;
        public static final double kMainLoopDt = 0.02;
        public static final double kFastLooperDt = 0.02;
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
        public static final int kKetteringReverseLimitSwitchTalonID = 3;

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

        public static final double kDriveKp = 7 * (0.1 * 1023.0) / (700);
        public static final double kDriveKd = 3 * kDriveKp;

        //TODO Turn In Place Scrub Tuning
        public static final double kTrackScrubFactor = 0.95;

        //Pure Pursuit Params
        public static final double kPathKX = 0.9;  // units/s per unit of error
        public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
        public static final double kPathMinLookaheadDistance = 24.0;  // inches

        //Talon PID Constants
        public static final double kMaxVel = 154.62;
        public static final double kMaxNativeVel = 3370.0;
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
        public static final double kCargoShipIntakeRollerOut = 0.3;
        public static final double kRocketLevelOneOutSpeed = 0.3;
        public static final double kRocketLevelTwoOutSpeed = 0.3;
        public static final double kDefaultIntakeRollerOutSpeed = 0.3;

        public static final double kArmOffset = -48.4;
        public static final double kFeedConstant = 0.20;


        public static final int kBookEnd_0 = kIsPracticeBot ? 4192 : -2078;
        public static final int kBookEnd_1 = kIsPracticeBot ? 1993 : 3790;
        public static final boolean kCrossOverZero = !kIsPracticeBot;
        public static final int kOffset = kIsPracticeBot ? -4192 : 0;
    }


    public static class HATCH_ARM {

        public static final boolean kHatchArmPlaceState = true;

        public static final boolean kHatchArmSensorPhase = false;
        public static final boolean kHatchArmMasterDirection = true;

        public static final double kMaxRawVel = 3085.0;

        public static final double kMotionMagicCruiseVel = kMaxRawVel * 0.9;
        public static final double kMotionMagicAccel = kMaxRawVel * 3;

        public static final double kMaxSafeCurrent = 150;

        public static final int kBookEnd_0 = kIsPracticeBot ? 1997 : 827;
        public static final int kBookEnd_1 = kIsPracticeBot ? -90 : 3790; //97
        public static final boolean kCrossOverZero = kIsPracticeBot ? true : true;
        public static final int kOffset = kIsPracticeBot ? -1997 : 0;
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

        static {
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(12400.0), new InterpolatingDouble(15.9));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(9362.0), new InterpolatingDouble(20.16));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(6958.0), new InterpolatingDouble(26.26));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(5122.0), new InterpolatingDouble(32.45));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(3423.0), new InterpolatingDouble(42.8));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(2155.0), new InterpolatingDouble(57.44));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(1464.0), new InterpolatingDouble(73.35));
            VISION.kAreaToDistVisionMap.put(new InterpolatingDouble(980.0), new InterpolatingDouble(94.09));
        }

    }


    public static class CONFIG {
        public static final int kPIDPrimary = 0;
        public static final int kPIDAuxilliaryTurn = 1;

        public static final int kDistanceSlot = 0;
        public static final int kTurningSlot = 1;

        public static final Map<TalonLoc, TalonSRXConfiguration> kConfigs = new HashMap<TalonLoc, TalonSRXConfiguration>();

        static {
            kConfigs.put(TalonLoc.Left, new TalonSRXConfiguration());
            kConfigs.put(TalonLoc.Right, new TalonSRXConfiguration());
            kConfigs.put(TalonLoc.Cargo_Arm, new TalonSRXConfiguration());
            kConfigs.put(TalonLoc.Hatch_Arm, new TalonSRXConfiguration());
            kConfigs.put(TalonLoc.Cargo_Intake, new TalonSRXConfiguration());
            for (TalonLoc loc : kConfigs.keySet()) {
                TalonSRXConfiguration tal = kConfigs.get(loc);
                tal.peakOutputForward = GENERAL.kMaxNominalOutput;
                tal.peakOutputReverse = -GENERAL.kMaxNominalOutput;
                tal.neutralDeadband = 0.0;
                tal.voltageCompSaturation = 12.0;
                tal.voltageMeasurementFilter = 32;
                tal.neutralDeadband = 0.0;
                tal.remoteSensorClosedLoopDisableNeutralOnLOS = true;
                tal.limitSwitchDisableNeutralOnLOS = true;
                tal.softLimitDisableNeutralOnLOS = true;
                tal.motionCurveStrength = 4;
                tal.forwardLimitSwitchSource = LimitSwitchSource.Deactivated;
                tal.reverseLimitSwitchSource = LimitSwitchSource.Deactivated;
                tal.forwardLimitSwitchNormal = LimitSwitchNormal.Disabled;
                tal.reverseLimitSwitchNormal = LimitSwitchNormal.Disabled;
                if (loc == TalonLoc.Left || loc == TalonLoc.Right) {
                    tal.motionCurveStrength = 8;
                    tal.velocityMeasurementPeriod = VelocityMeasPeriod.Period_25Ms;
                    tal.velocityMeasurementWindow = 8;
                    //General Velocity/Motion Magic
                    tal.slot0.kP = DRIVE.kDriveKp;
                    tal.slot0.kD = DRIVE.kDriveKd;
                    tal.slot0.kF = 1023.0 / DRIVE.kMaxNativeVel;
                    tal.slot0.closedLoopPeakOutput = 0.3;
                    tal.slot0.allowableClosedloopError = 100;
                    //Motion Magic Turning
                    tal.slot1.kP = 4.4;
                    tal.slot1.kI = 0.3;
                    tal.slot1.kD = tal.slot1.kP * 34;
                    tal.slot1.kF = 1023.0 / DRIVE.kMaxNativeVel;
                    tal.slot1.integralZone = 100;
                    tal.slot1.maxIntegralAccumulator = 50;
                    tal.slot1.closedLoopPeakOutput = 0.7;
                    tal.slot1.allowableClosedloopError = 10;
                    //~3deg
                    tal.auxPIDPolarity = false;
                    tal.motionCruiseVelocity = (int) (DRIVE.kMaxNativeVel * 0.9);
                    tal.motionAcceleration = (int) (tal.motionCruiseVelocity * 0.5);
                } else if (loc == TalonLoc.Cargo_Arm || loc == TalonLoc.Hatch_Arm) {
                    tal.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
                    tal.reverseLimitSwitchSource = LimitSwitchSource.RemoteTalonSRX;
                    tal.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
                    tal.forwardSoftLimitEnable = true;
                    tal.reverseSoftLimitEnable = true;
                    tal.clearPositionOnLimitR = false;//TODO fix
                }

                if (loc == TalonLoc.Left) {
                    tal.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
                    tal.primaryPID.selectedFeedbackCoefficient = 1.0;
                } else if (loc == TalonLoc.Right) {
                    tal.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
                    tal.primaryPID.selectedFeedbackCoefficient = 1.0;

                   /* tal.primaryPID.selectedFeedbackSensor = FeedbackDevice.SensorSum;
                    tal.primaryPID.selectedFeedbackCoefficient = 0.50; */

                    tal.auxiliaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor1;
                    tal.auxiliaryPID.selectedFeedbackCoefficient = 1.0;

                    tal.remoteFilter0.remoteSensorDeviceID = CAN.kDriveLeftMasterTalonID;
                    tal.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonSRX_SelectedSensor;

                    tal.remoteFilter1.remoteSensorDeviceID = CAN.kLeftCargoIntakeTalonID;
                    tal.remoteFilter1.remoteSensorSource = RemoteSensorSource.GadgeteerPigeon_Yaw;

                    tal.sum0Term = FeedbackDevice.RemoteSensor0;
                    tal.sum1Term = FeedbackDevice.CTRE_MagEncoder_Relative;
                } else if (loc == TalonLoc.Cargo_Arm) {
                    tal.reverseLimitSwitchDeviceID = CAN.kRightCargoIntakeTalonID;
                    tal.forwardSoftLimitThreshold = (int) MkMath.degreesToNativeUnits(180);
                    tal.reverseSoftLimitThreshold = 0;
                    tal.motionCruiseVelocity = (int) (CARGO_ARM.kMaxRawVel);
                    tal.motionAcceleration = (int) (CARGO_ARM.kMaxRawVel * 10);
                    tal.slot0.kP = (39.0 * ((0.1 * 1023.0) / (1600))); //7.5 deg or 1390 units
                    tal.slot0.kI = tal.slot0.kP / 210;
                    tal.slot0.kD = tal.slot0.kP * 40;
                    tal.slot0.kF = 1023.0 / CARGO_ARM.kMaxRawVel;
                    tal.slot0.integralZone = 200;
                    tal.slot0.maxIntegralAccumulator = 300;
                    tal.motionCurveStrength = 3;
                } else if (loc == TalonLoc.Hatch_Arm) {
                    tal.reverseLimitSwitchDeviceID = CAN.kKetteringReverseLimitSwitchTalonID;
                    tal.forwardSoftLimitThreshold = (int) MkMath.degreesToNativeUnits(188);
                    tal.reverseSoftLimitThreshold = 0;
                    tal.motionCruiseVelocity = (int) (HATCH_ARM.kMotionMagicCruiseVel);
                    tal.motionAcceleration = (int) (HATCH_ARM.kMotionMagicAccel);
                    tal.slot0.kP = 30.0 * ((0.1 * 1023.0) / (1600)); //7.5 deg or 1390 units
                    tal.slot0.kI = (tal.slot0.kP / 500) * 0.0;
                    tal.slot0.kD = tal.slot0.kP * 40;
                    tal.slot0.kF = (1023.0 / Constants.HATCH_ARM.kMaxRawVel);
                    tal.slot0.maxIntegralAccumulator = 0;
                    tal.slot0.integralZone = 0;
                    tal.motionCurveStrength = 7;
                    //  tal.nominalOutputForward = 0.01;
                    // tal.nominalOutputReverse = -0.01;
                }
            }
        }
    }
}
