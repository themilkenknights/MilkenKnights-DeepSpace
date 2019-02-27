package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import frc.robot.Constants.CONFIG;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.drivers.CT;
import frc.robot.lib.drivers.MkGyro;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Pose2dWithCurvature;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Twist2d;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.lib.trajectory.timing.TimedState;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.ReflectingCSVWriter;
import frc.robot.paths.DriveMotionPlanner;
import frc.robot.paths.Kinematics;
import frc.robot.paths.RobotState;

public class Drive extends Subsystem {

    private final MkTalon mLeftDrive, mRightDrive;
    private final PigeonIMU mPigeon;
    private final MkGyro navX;
    public PeriodicIO mPeriodicIO;
    public DriveControlState mDriveControlState;
    private DriveMotionPlanner mMotionPlanner;
    private Rotation2d mGyroOffset = Rotation2d.identity();
    private boolean mOverrideTrajectory, mIsOnTarget = false;
    private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
    private double left_encoder_prev_distance_, right_encoder_prev_distance_, mDesiredTurnAngle = 0.0;
    private NetworkTableEntry mState, mStatus, mFusedHeading, mXErr, mYErr, mThetaErr, mGyroHeading;

    private Drive() {
        ShuffleboardTab mDriveTab = Shuffleboard.getTab("Drive");
        mState = mDriveTab.add("State", "").getEntry();
        mStatus = mDriveTab.add("Status", false).getEntry();
        mFusedHeading = mDriveTab.add("Fused Heading", 0.0).getEntry();
        mXErr = mDriveTab.add("x err", 0.0).getEntry();
        mYErr = mDriveTab.add("y err", 0.0).getEntry();
        mThetaErr = mDriveTab.add("theta err", 0.0).getEntry();
        mGyroHeading = mDriveTab.add("Gyro Heading", 0.0).getEntry();
        mDriveControlState = DriveControlState.OPEN_LOOP;
        mPeriodicIO = new PeriodicIO();
        mLeftDrive = new MkTalon(Constants.CAN.kDriveLeftMasterTalonID, Constants.CAN.kDriveLeftSlaveVictorID, TalonLoc.Left, mDriveTab);
        mRightDrive = new MkTalon(Constants.CAN.kDriveRightMasterTalonID, Constants.CAN.kDriveRightSlaveVictorID, TalonLoc.Right, mDriveTab);
        mPigeon = new PigeonIMU(CargoArm.getInstance().mIntakeTalon.masterTalon);
        CT.RE(mPigeon.configFactoryDefault(GENERAL.kLongCANTimeoutMs));
        CT.RE(mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 10, GENERAL.kLongCANTimeoutMs));
        CT.RE(mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 3, GENERAL.kLongCANTimeoutMs));
        CT.RE(mPigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 3, GENERAL.kLongCANTimeoutMs));
        navX = new MkGyro(Port.kMXP);
        mMotionPlanner = new DriveMotionPlanner();
    }

    public static Drive getInstance() {
        return InstanceHolder.mInstance;
    }

    /**
     * Step 1: Read inputs from Talon and NavX
     */
    @Override public synchronized void readPeriodicInputs(double timestamp) {
        mPeriodicIO.timestamp = Timer.getFPGATimestamp();
        mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(mPigeon.getFusedHeading()).rotateBy(mGyroOffset);
        mPeriodicIO.leftPos = mLeftDrive.getPosition();
        mPeriodicIO.rightPos = mRightDrive.getPosition();
        mPeriodicIO.leftVel = mLeftDrive.getVelocity();
        mPeriodicIO.rightVel = mRightDrive.getVelocity();
    }

    /**
     * Periodic update after read. Used to update odometry and path setpoints
     *
     * @param timestamp In Seconds Since Code Start
     */
    @Override public synchronized void onQuickLoop(double timestamp) {
        stateEstimator(timestamp);
        switch (mDriveControlState) {
            case OPEN_LOOP:
            case MOTION_MAGIC:
            case PIGEON_SERVO:
                break;
            case PATH_FOLLOWING:
                updatePathFollower();
                break;
            default:
                Logger.logErrorWithTrace("Unexpected drive control state: " + mDriveControlState);
                break;
        }

    }

    /*
    Update path setpoints and parameters
     */
    private synchronized void updatePathFollower() {
        if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            final double now = Timer.getFPGATimestamp();

            DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

            // DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0, demand.right_feedforward_voltage / 12.0);

            mPeriodicIO.error = mMotionPlanner.error();
            mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

            if (!mOverrideTrajectory) {
                setVelocity(new DriveSignal(MkMath.radiansPerSecondToTicksPer100ms(output.left_velocity), MkMath.radiansPerSecondToTicksPer100ms(output.right_velocity)),
                    new DriveSignal(output.left_feedforward_voltage / 12.0, output.right_feedforward_voltage / 12.0));

                mPeriodicIO.left_accel = MkMath.radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
                mPeriodicIO.right_accel = MkMath.radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
            } else {
                setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
                mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
            }
        } else {
            Logger.logError("Drive is not in path following state");
        }
    }

    /**
     * Write setpoints to Talons and Victors left_demand and right_demand are always in Talon Native Units or Talon Native Units Per 100ms left_feedforward and
     * right_feedforward are in Percent Output [-1,1] and are added to the Talon SRX Closed Loop Output
     */
    @Override public synchronized void writePeriodicOutputs(double timestamp) {
        if (mDriveControlState == DriveControlState.OPEN_LOOP) {
            //mLeftDrive.set(ControlMode.Velocity, mPeriodicIO.left_demand * DRIVE.kMaxNativeVel, mPeriodicIO.brake_mode);
            //mRightDrive.set(ControlMode.Velocity, mPeriodicIO.right_demand * DRIVE.kMaxNativeVel, mPeriodicIO.brake_mode);
            mLeftDrive.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, mPeriodicIO.brake_mode);
            mRightDrive.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, mPeriodicIO.brake_mode);
        } else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            mLeftDrive.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
                mPeriodicIO.left_feedforward + DRIVE.kDriveKd * mPeriodicIO.left_accel / 1023.0, NeutralMode.Brake);
            mRightDrive.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
                mPeriodicIO.right_feedforward + DRIVE.kDriveKd * mPeriodicIO.right_accel / 1023.0, NeutralMode.Brake);
        } else if (mDriveControlState == DriveControlState.MOTION_MAGIC) {
            mLeftDrive.set(ControlMode.MotionMagic, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.left_feedforward, mPeriodicIO.brake_mode);
            mRightDrive.set(ControlMode.MotionMagic, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.right_feedforward, mPeriodicIO.brake_mode);
        } else if (mDriveControlState == DriveControlState.PIGEON_SERVO) {
            mRightDrive.set(ControlMode.MotionMagic, mPeriodicIO.right_demand, DemandType.AuxPID, mPeriodicIO.right_feedforward, NeutralMode.Brake);
        } else {
            Logger.logErrorWithTrace("Unexpected drive control state: " + mDriveControlState);
        }
    }

    /**
     * Update Shuffleboard and Log to CSV
     */
    public synchronized void outputTelemetry(double timestamp) {
        mLeftDrive.updateShuffleboard();
        mRightDrive.updateShuffleboard();
        mState.setString(mDriveControlState.toString());
        mStatus.setBoolean(driveStatus());
        mFusedHeading.setDouble(mPigeon.getFusedHeading());
        mXErr.setDouble(mPeriodicIO.error.getTranslation().x());
        mYErr.setDouble(mPeriodicIO.error.getTranslation().y());
        mThetaErr.setDouble(mPeriodicIO.error.getRotation().getDegrees());
        if (getHeading() != null) {
            mGyroHeading.setDouble(getHeading().getDegrees());
        }
        if (mCSVWriter != null) {
            mCSVWriter.add(mPeriodicIO);
            mCSVWriter.write();
        }
    }

    /**
     * Stop drive motors and save log to CSV File
     */
    @Override public void onStop(double timestamp) {
        setOpenLoop(DriveSignal.BRAKE);
        if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }
    }

    /**
     * Controls Drive motors in PercentOutput Mode (No closed loop control)
     */
    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != DriveControlState.OPEN_LOOP) {
            Logger.logMarker("Switching to open loop");
            configNormalDrive();
            mDriveControlState = DriveControlState.OPEN_LOOP;
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = 0.0;
        mPeriodicIO.brake_mode = NeutralMode.Brake;
    }

    /**
     * @param signal Left/Right Position in inches
     * @param feedforward Left/Right arbitrary feedforward (Percent Output, [-1,1])
     */
    public synchronized void updateMotionMagicPositionSetpoint(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.MOTION_MAGIC) {
            Logger.logMarker("Switching to Motion Magic");
            configNormalDrive();
            mDriveControlState = DriveControlState.MOTION_MAGIC;
        }
        mPeriodicIO.left_demand = MkMath.InchesToNativeUnits(signal.getLeft());
        mPeriodicIO.right_demand = MkMath.InchesToNativeUnits(signal.getRight());
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
        mPeriodicIO.brake_mode = NeutralMode.Brake;
    }

    public synchronized DriveSignal setMotionMagicDeltaSetpoint(DriveSignal signal, DriveSignal feedforward) {
        DriveSignal newSig = new DriveSignal(signal.getLeft() + mPeriodicIO.leftPos, signal.getRight() + mPeriodicIO.rightPos, signal.getBrakeMode());
        updateMotionMagicPositionSetpoint(newSig, feedforward);
        return newSig;
    }

    /**
     * Calculate position deltas and read gyro angle to update odometry information.
     *
     * @param timestamp Current FPGA Time since code start
     */
    private synchronized void stateEstimator(double timestamp) {
        final double left_distance = mPeriodicIO.leftPos;
        final double right_distance = mPeriodicIO.rightPos;
        final double delta_left = left_distance - left_encoder_prev_distance_;
        final double delta_right = right_distance - right_encoder_prev_distance_;
        final Rotation2d gyro_angle = getHeading();
        final Twist2d odometry_velocity = RobotState.getInstance().generateOdometryFromSensors(delta_left, delta_right, gyro_angle);
        final Twist2d predicted_velocity = Kinematics.forwardKinematics(mPeriodicIO.leftVel, mPeriodicIO.leftVel);
        RobotState.getInstance().addObservations(timestamp, odometry_velocity, predicted_velocity);
        left_encoder_prev_distance_ = left_distance;
        right_encoder_prev_distance_ = right_distance;
    }

    /**
     * Controls Drivetrain in Closed-loop velocity Mode Method sets Talons in Native Units per 100ms
     *
     * @param signal An object that contains left and right velocities (inches per sec)
     */
    private synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
        if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            Logger.logMarker("Switching to Velocity");
            configNormalDrive();
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_feedforward = feedforward.getLeft();
        mPeriodicIO.right_feedforward = feedforward.getRight();
        mPeriodicIO.brake_mode = signal.getBrakeMode();
    }

    /**
     * Use Limelight to find Distance and Angle. Drive and turn to target using primary and aux PID on Talons
     *
     * @param dist Distance in Inches to Drive (Motion Magic)
     * @param angle Angle to serve to using Pigeon
     */
    public synchronized void setDistanceAndAngle(double dist, double angle) {
        if (mDriveControlState != DriveControlState.PIGEON_SERVO) {
            Logger.logMarker("Switching to Pigeon Servo");
            configHatchVision();
            mDriveControlState = DriveControlState.PIGEON_SERVO;
        }

        mPeriodicIO.left_demand = 0.0;
        mPeriodicIO.right_demand = MkMath.InchesToNativeUnits(dist + mPeriodicIO.rightPos);
        mPeriodicIO.left_feedforward = 0.0;
        mPeriodicIO.right_feedforward = getHeadingNormalized() - angle;
        mPeriodicIO.brake_mode = NeutralMode.Brake;
    }

    /**
     * @param angle Desired relative angle in degrees to turn to.
     */
    public synchronized void setTurnInPlaceHeading(double angle) {
        mDesiredTurnAngle = angle;
        updateTurnToHeading();
    }

    /**
     * Read current rotation relative to desired angle and update motion magic position setpoints accordingly.
     * Stop motion when tolerances are met.
     */
    public synchronized void updateTurnToHeading() {
        final Rotation2d field_to_robot = RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation();
        // Figure out the rotation necessary to turn to face the goal.
        final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(Rotation2d.fromDegrees(mDesiredTurnAngle));
        // Check if we are on target
        if (Math.abs(robot_to_target.getDegrees()) < Constants.DRIVE.kGoalPosTolerance && Math.abs(mPeriodicIO.leftVel) < Constants.DRIVE.kGoalVelTolerance
            && Math.abs(mPeriodicIO.rightVel) < Constants.DRIVE.kGoalVelTolerance) {
            // Use the current setpoint and base lock.
            mIsOnTarget = true;
            updateMotionMagicPositionSetpoint(new DriveSignal(mPeriodicIO.leftPos, mPeriodicIO.rightPos, NeutralMode.Brake), DriveSignal.BRAKE);
            return;
        }
        Kinematics.DriveVelocity wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
        updateMotionMagicPositionSetpoint(new DriveSignal(wheel_delta.left + mPeriodicIO.leftPos, wheel_delta.right + mPeriodicIO.rightPos, NeutralMode.Brake),
            DriveSignal.BRAKE);
    }

    private void configHatchVision() {
        if (mDriveControlState != DriveControlState.PIGEON_SERVO) {
            mRightDrive.masterTalon.configClosedLoopPeakOutput(CONFIG.kDistanceSlot, 0.5, 0);
            mRightDrive.masterTalon.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, CONFIG.kPIDPrimary, 0);
            mRightDrive.masterTalon.configSelectedFeedbackCoefficient(0.5, CONFIG.kPIDPrimary, 0);
            mLeftDrive.masterTalon.follow(mRightDrive.masterTalon, FollowerType.AuxOutput1);
        }
    }

    private void configNormalDrive() {
        if (mDriveControlState == DriveControlState.PIGEON_SERVO) {
            mRightDrive.masterTalon.configClosedLoopPeakOutput(CONFIG.kDistanceSlot, 1.0, 0);
            mRightDrive.masterTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, CONFIG.kPIDPrimary, 0);
            mRightDrive.masterTalon.configSelectedFeedbackCoefficient(1.0, CONFIG.kPIDPrimary, 0);
            setOpenLoop(DriveSignal.BRAKE);
        }
    }

    /**
     * Passes the trajectory to the drive motion planner and configures drive control state
     */
    public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
        if (mMotionPlanner != null) {
            mOverrideTrajectory = false;
            mMotionPlanner.reset();
            mMotionPlanner.setTrajectory(trajectory);
            mDriveControlState = DriveControlState.PATH_FOLLOWING;
        }
    }

    /**
     * Clear mag encoder position and local distance counter and start logging if appropriate
     */
    @Override public void autonomousInit(double timestamp) {
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
        navX.zeroYaw();
        zeroPigeon();
        left_encoder_prev_distance_ = 0;
        right_encoder_prev_distance_ = 0;
        if (mCSVWriter == null && Constants.LOG.kDriveCSVLogging) {
            mCSVWriter = new ReflectingCSVWriter<>("DRIVE-LOGS", PeriodicIO.class);
        }
        setHeading(Rotation2d.identity());
        mLeftDrive.masterTalon.setSelectedSensorPosition(0);
        mRightDrive.masterTalon.setSelectedSensorPosition(0);
    }

    public void teleopInit(double timestamp) {
        if (mCSVWriter == null && Constants.LOG.kDriveCSVLogging) {
            mCSVWriter = new ReflectingCSVWriter<>("DRIVE-LOGS", PeriodicIO.class);
        }
    }

    public boolean checkSystem() {
        boolean driveCheck = mLeftDrive.checkSystem() & mRightDrive.checkSystem();
        driveCheck &= mRightDrive.checkDriveDeltas();
        if (driveCheck) {
            Logger.logMarker("Drive Test Success");
        }
        if (!navX.isConnected()) {
            Logger.logErrorWithTrace("FAILED - NAVX DISCONNECTED");
            driveCheck = false;
        } else {
            Logger.logMarker("NavX Connected");
        }
        mLeftDrive.resetConfig();
        mRightDrive.resetConfig();
        return driveCheck;
    }

    /**
     * Check if the trajectory is finished
     */
    public synchronized boolean isDoneWithTrajectory() {
        if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            return false;
        }
        return mMotionPlanner.isDone() || mOverrideTrajectory;
    }

    /**
     * @return If both SRX Mag Encoders and the NavX is connected
     */
    private synchronized boolean driveStatus() {
        return mLeftDrive.isEncoderConnected() && mRightDrive.isEncoderConnected() && navX.isConnected();
    }

    public synchronized boolean isTurnDone() {
        return mIsOnTarget;
    }

    private synchronized Rotation2d getHeading() {
        return mPeriodicIO.gyro_heading;
    }

    /**
     * Zero heading by adding a software offset
     */
    public synchronized void setHeading(Rotation2d heading) {
        Logger.logMarker("SET HEADING: " + heading.getDegrees());
        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(-navX.getAngle()).inverse());
        Logger.logMarker("Gyro offset: " + mGyroOffset.getDegrees());
        mPeriodicIO.gyro_heading = heading;
    }

    /**
     * Zero all pigeon values
     */
    private void zeroPigeon() {
        CT.RE(mPigeon.setFusedHeading(0, GENERAL.kLongCANTimeoutMs));
        CT.RE(mPigeon.setYaw(0, GENERAL.kLongCANTimeoutMs));
        CT.RE(mPigeon.setAccumZAngle(0, GENERAL.kLongCANTimeoutMs));
    }

    /**
     * @return current fused heading from navX
     */
    public double getHeadingDeg() {
        return mPeriodicIO.gyro_heading.getDegrees();
    }

    /**
     * Negative is Left (Counterclockwise)
     * Positive is Right (Clockwise)
     * From -180 to 180 deg
     * //TODO Verify Method Is Correct With Pigeon
     */
    public double getHeadingNormalized() {
        if (mPeriodicIO.gyro_heading.getDegrees() < 180.0) {
            return -mPeriodicIO.gyro_heading.getDegrees();
        } else {
            return 180 - (mPeriodicIO.gyro_heading.getDegrees() - 180);
        }
    }

    public double getRawHeading(){
        return mPigeon.getFusedHeading();
    }

    public double getYaw() {
        return navX.getYaw();
    }

    public boolean isMotionMagicFinished() {
        return mLeftDrive.getError() < DRIVE.kGoalPosTolerance && mRightDrive.getError() < DRIVE.kGoalPosTolerance;
    }

    public enum DriveControlState {
        OPEN_LOOP, PATH_FOLLOWING, // velocity PID control
        MOTION_MAGIC, PIGEON_SERVO
    }


    private static class InstanceHolder {

        private static final Drive mInstance = new Drive();
    }


    public static class PeriodicIO {

        public double timestamp;
        //Inputs
        public double leftPos;
        public double rightPos;
        public double leftVel;
        public double rightVel;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();
        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public NeutralMode brake_mode;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }

}
