package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CONFIG;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.MISC;
import frc.robot.lib.drivers.CT;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.geometry.*;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.*;
import frc.robot.lib.util.trajectory.Path;
import frc.robot.lib.util.trajectory.PathFollower;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class Drive extends Subsystem {
  private final MkTalon mLeftDrive, mRightDrive;
  public PeriodicIO mPeriodicIO;
  private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
  private Rotation2d mGyroOffset = Rotation2d.identity();
  private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
  private double left_encoder_prev_distance_, right_encoder_prev_distance_ = 0.0;
  private NetworkTableEntry mState, mStatus, mFusedHeading, mGyroHeading, mAvgDist;
  private PigeonIMU mPigeon;
  private PathFollower pathFollower = null;
  private TrajectoryStatus leftStatus;
  private TrajectoryStatus rightStatus;
  private DriveSignal currentSetpoint;
  private double lastAngle = 0;

  private Drive() {
    ShuffleboardTab mDriveTab = Shuffleboard.getTab("Drive");
    mState = mDriveTab.add("State", "").getEntry();
    mStatus = mDriveTab.add("Status", false).getEntry();
    mFusedHeading = mDriveTab.add("Fused Heading", 0.0).getEntry();
    mGyroHeading = mDriveTab.add("Gyro Heading", 0.0).getEntry();
    mAvgDist = mDriveTab.add("Avg Dist", 0.0).getEntry();
    mPeriodicIO = new PeriodicIO();
    mLeftDrive = new MkTalon(Constants.CAN.kDriveLeftMasterTalonID, Constants.CAN.kDriveLeftSlaveVictorID, TalonLoc.Left, mDriveTab);
    mRightDrive = new MkTalon(Constants.CAN.kDriveRightMasterTalonID, Constants.CAN.kDriveRightSlaveVictorID, TalonLoc.Right, mDriveTab);
    mPigeon = CargoArm.getInstance().getPigeon();
    leftStatus = TrajectoryStatus.NEUTRAL;
    rightStatus = TrajectoryStatus.NEUTRAL;
    currentSetpoint = DriveSignal.BRAKE;
  }

  public static Drive getInstance() {
    return InstanceHolder.mInstance;
  }

  /**
   * Step 1: Read inputs from Talon and Pigeon
   */
  @Override
  public synchronized void readPeriodicInputs(double timestamp) {
    double fusedHeading = mPigeon.getFusedHeading();
    mPeriodicIO.timestamp = Timer.getFPGATimestamp();
    mPeriodicIO.fusedHeading = fusedHeading;
    mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(fusedHeading).rotateBy(mGyroOffset);
    mPeriodicIO.leftPos = mLeftDrive.getPosition();
    mPeriodicIO.rightPos = mRightDrive.getPosition();
    mPeriodicIO.leftVel = mLeftDrive.getVelocity();
    mPeriodicIO.rightVel = mRightDrive.getVelocity();
  }

  @Override
  public synchronized void onQuickLoop(double timestamp) {
    switch (mDriveControlState) {
      case OPEN_LOOP:
      case MOTION_MAGIC:
      case PIGEON_SERVO:
        break;
      case VELOCITY_SETPOINT:
        updatePathFollower();
        break;
      default:
        Logger.logErrorWithTrace("Unknown Drive Control State");
        break;
    }
  }

  /**
   * Write setpoints to Talons and Victors left_demand and right_demand are always in Talon Native
   * Units or Talon Native Units Per 100ms left_feedforward and right_feedforward are in Percent
   * Output [-1,1] and are added to the Talon SRX Closed Loop Output
   */
  @Override
  public synchronized void writePeriodicOutputs(double timestamp) {
    if (mDriveControlState == DriveControlState.OPEN_LOOP) {
      mLeftDrive.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, mPeriodicIO.brake_mode);
      mRightDrive.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, mPeriodicIO.brake_mode);
    } else if (mDriveControlState == DriveControlState.MOTION_MAGIC) {
      mLeftDrive.set(ControlMode.MotionMagic, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.left_feedforward, mPeriodicIO.brake_mode);
      mRightDrive.set(ControlMode.MotionMagic, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.right_feedforward, mPeriodicIO.brake_mode);
    } else if (mDriveControlState == DriveControlState.PIGEON_SERVO) {
      mRightDrive.set(ControlMode.MotionMagic, mPeriodicIO.right_demand, DemandType.AuxPID, mPeriodicIO.right_feedforward, mPeriodicIO.brake_mode);
    } else if (mDriveControlState == DriveControlState.VELOCITY_SETPOINT) {
      mLeftDrive.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.left_feedforward, mPeriodicIO.brake_mode);
      mRightDrive.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.right_feedforward, mPeriodicIO.brake_mode);
    } else {
      Logger.logErrorWithTrace("Unexpected drive control state: " + mDriveControlState);
    }
  }

  /*
   * Periodic update after read. Used to update odometry and path setpoints
   *
   * @param timestamp In Seconds Since Code Start
   *
   * @Override public synchronized void onQuickLoop(double timestamp) { stateEstimator(timestamp);
   * switch (mDriveControlState) { case OPEN_LOOP: case MOTION_MAGIC: case PIGEON_SERVO: break;
   * default: Logger.logErrorWithTrace("Unexpected drive control state: " + mDriveControlState);
   * break; } }
   */
  /**
   * Update Shuffleboard and Log to CSV
   */
  public synchronized void outputTelemetry(double timestamp) {
    mLeftDrive.updateShuffleboard();
    mRightDrive.updateShuffleboard();
    mFusedHeading.setDouble(mPeriodicIO.fusedHeading);
    mAvgDist
        .setDouble((mLeftDrive.masterTalon.getSensorCollection().getQuadraturePosition() + mRightDrive.masterTalon.getSensorCollection().getQuadraturePosition()) / 2.0);
    if (getHeading() != null) {
      mGyroHeading.setDouble(getHeading().getDegrees());
    }
    if (mCSVWriter != null && MISC.kDriveCSVLogging) {
      mCSVWriter.add(mPeriodicIO);
      mCSVWriter.write();
    }
    mState.setString(mDriveControlState.toString());
    mStatus.setBoolean(driveStatus());
    SmartDashboard.putNumber("Aux Error", mRightDrive.getError(1));
    SmartDashboard.putNumber("Aux Target", mRightDrive.getTarget(1));
    SmartDashboard.putNumber("Aux Pos", mRightDrive.getPosition(1));
    SmartDashboard.putNumber("Main Target", mRightDrive.getTarget(0));
    SmartDashboard.putNumber("Left Enc", MkMath.nativeUnitsToDegrees(mLeftDrive.masterTalon.getSensorCollection().getQuadraturePosition()));
    SmartDashboard.putNumber("Right Enc", MkMath.nativeUnitsToDegrees(mRightDrive.masterTalon.getSensorCollection().getQuadraturePosition()));
  }

  public void teleopInit(double timestamp) {
    zero();
  }

  /**
   * Clear mag encoder position and local distance counter and start logging if appropriate
   */
  @Override
  public void autonomousInit(double timestamp) {
    zero();
  }

  /**
   * Stop drive motors and save log to CSV File
   */
  @Override
  public void onStop(double timestamp) {
    setOpenLoop(DriveSignal.BRAKE);
    if (mCSVWriter != null && MISC.kDriveCSVLogging) {
      mCSVWriter.flush();
      mCSVWriter = null;
    }
  }

  @Override
  public void onRestart(double timestamp) {
    mLeftDrive.checkForErrorInit();
    mRightDrive.checkForErrorInit();
  }

  public boolean checkSystem() {
    boolean driveCheck = mLeftDrive.checkSystem() & mRightDrive.checkSystem();
    driveCheck &= mRightDrive.checkDriveDeltas();
    if (driveCheck) {
      Logger.logMarker("Drive Test Success");
    }
    mLeftDrive.resetConfig();
    mRightDrive.resetConfig();
    return driveCheck;
  }

  /**
   * Controls Drive motors in PercentOutput Mode (No closed loop control)
   */
  public synchronized void setOpenLoop(DriveSignal signal) {
    if (mDriveControlState != DriveControlState.OPEN_LOOP) {
      Logger.logMarker("Switching to open loop");
      mPeriodicIO.left_demand = 0.0;
      mPeriodicIO.right_demand = 0.0;
      mPeriodicIO.left_feedforward = 0.0;
      mPeriodicIO.right_feedforward = 0.0;
      configNormalDrive();
      mDriveControlState = DriveControlState.OPEN_LOOP;
    }
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = 0.0;
    mPeriodicIO.right_feedforward = 0.0;
    mPeriodicIO.brake_mode = NeutralMode.Brake;
  }

  private synchronized void configNormalDrive() {
    if (mDriveControlState == DriveControlState.PIGEON_SERVO) {
      mRightDrive.masterTalon.configClosedLoopPeakOutput(CONFIG.kDistanceSlot, 1.0, 0);
    }
  }

  private void zero() {
    left_encoder_prev_distance_ = 0;
    right_encoder_prev_distance_ = 0;
    mLeftDrive.zeroEncoder();
    mRightDrive.zeroEncoder();
    zeroPigeon();
    RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
    setHeading(Rotation2d.identity());
    if (mCSVWriter == null && MISC.kDriveCSVLogging) {
      mCSVWriter = new ReflectingCSVWriter<>("DRIVE-LOGS", PeriodicIO.class);
    }
  }

  /**
   * Zero all pigeon values
   */
  public void zeroPigeon() {
    CT.RE(mPigeon.setFusedHeading(0, 0));
    CT.RE(mPigeon.setYaw(0, 0));
    CT.RE(mPigeon.setAccumZAngle(0, 0));
  }

  private synchronized Rotation2d getHeading() {
    return mPeriodicIO.gyro_heading;
  }

  /**
   * Zero heading by adding a software offset
   */
  public synchronized void setHeading(Rotation2d heading) {
    Logger.logMarker("SET HEADING: " + heading.getDegrees());
    mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(mPigeon.getFusedHeading()).inverse());
    Logger.logMarker("Gyro offset: " + mGyroOffset.getDegrees());
    mPeriodicIO.gyro_heading = heading;
  }

  /**
   * @return If both SRX Mag Encoders are connected
   */
  private synchronized boolean driveStatus() {
    return mLeftDrive.isEncoderConnected() && mRightDrive.isEncoderConnected();
  }

  public synchronized DriveSignal setMotionMagicDeltaSetpoint(DriveSignal signal, DriveSignal feedforward) {
    DriveSignal newSig = new DriveSignal(signal.getLeft() + mPeriodicIO.leftPos, signal.getRight() + mPeriodicIO.rightPos, signal.getBrakeMode());
    updateMotionMagicPositionSetpoint(newSig, feedforward);
    return newSig;
  }

  /**
   * @param signal Left/Right Position in inches
   * @param feedforward Left/Right arbitrary feedforward (Percent Output, [-1,1])
   */
  public synchronized void updateMotionMagicPositionSetpoint(DriveSignal signal, DriveSignal feedforward) {
    if (mDriveControlState != DriveControlState.MOTION_MAGIC) {
      Logger.logMarker("Switching to Motion Magic");
      mPeriodicIO.left_demand = 0.0;
      mPeriodicIO.right_demand = 0.0;
      mPeriodicIO.left_feedforward = 0.0;
      mPeriodicIO.right_feedforward = 0.0;
      configNormalDrive();
      mDriveControlState = DriveControlState.MOTION_MAGIC;
    }
    mPeriodicIO.left_demand = MkMath.InchesToNativeUnits(signal.getLeft());
    mPeriodicIO.right_demand = MkMath.InchesToNativeUnits(signal.getRight());
    mPeriodicIO.left_feedforward = feedforward.getLeft();
    mPeriodicIO.right_feedforward = feedforward.getRight();
    mPeriodicIO.brake_mode = NeutralMode.Brake;
  }

  /**
   * Use Limelight to find Distance and Angle. Drive and turn to target using primary and aux PID on
   * Talons
   *
   * @param dist Distance in Inches to Drive (Motion Magic)
   * @param angle Angle to serve to using Pigeon
   */
  public synchronized void setDistanceAndAngle(double dist, double angle) {
    if (mDriveControlState != DriveControlState.PIGEON_SERVO) {
      mPeriodicIO.left_demand = 0.0;
      mPeriodicIO.right_demand = 0.0;
      mPeriodicIO.left_feedforward = 0.0;
      mPeriodicIO.right_feedforward = 0.0;
      configHatchVision();
      mDriveControlState = DriveControlState.PIGEON_SERVO;
      Logger.logMarker("Switching to Pigeon Servo");
    }
    mPeriodicIO.left_demand = 0.0;
    mPeriodicIO.right_demand = MkMath.InchesToNativeUnits(dist + mPeriodicIO.rightPos);
    mPeriodicIO.left_feedforward = 0.0;
    mPeriodicIO.right_feedforward = MkMath.degreesToPigeonNativeUnits(mPeriodicIO.fusedHeading + angle);
    mPeriodicIO.brake_mode = NeutralMode.Brake;
    // System.out.println(MkMath.degreesToPigeonNativeUnits(mPeriodicIO.fusedHeading + angle));
  }

  public synchronized void configHatchVision() {
    if (mDriveControlState != DriveControlState.PIGEON_SERVO) {
      mRightDrive.masterTalon.configClosedLoopPeakOutput(CONFIG.kDistanceSlot, 0.5, 0);
      mLeftDrive.masterTalon.follow(mRightDrive.masterTalon, FollowerType.AuxOutput1);
    }
  }

  /**
   * @return The distance from the target when servoing with the Pigeon
   */
  public double getVisionServoError(double dist) {
    return Math.abs((mPeriodicIO.rightPos / 2) - dist);
  }

  public void setVelocitySetpointNormal(DriveSignal sig) {
    setVelocity(new DriveSignal(sig.getLeft() * DRIVE.kMaxNativeVel, sig.getRight() * DRIVE.kMaxNativeVel), DriveSignal.BRAKE);
  }

  /**
   * Controls Drivetrain in Closed-loop velocity Mode Method sets Talons in Native Units per 100ms
   *
   * @param signal An object that contains left and right velocities (inches per sec)
   */
  public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
    if (mDriveControlState != DriveControlState.VELOCITY_SETPOINT) {
      Logger.logMarker("Switching to Velocity");
      mPeriodicIO.left_demand = 0.0;
      mPeriodicIO.right_demand = 0.0;
      mPeriodicIO.left_feedforward = 0.0;
      mPeriodicIO.right_feedforward = 0.0;
      configNormalDrive();
      mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
    }
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = feedforward.getLeft();
    mPeriodicIO.right_feedforward = feedforward.getRight();
    mPeriodicIO.brake_mode = signal.getBrakeMode();
  }

  /**
   * @return Pitch from the Pigeon IMU [-90 to 90deg]
   */
  public double getPitch() {
    double[] arr = new double[3];
    mPigeon.getYawPitchRoll(arr);
    return arr[2];
  }

  public boolean isMotionMagicFinished() {
    return Math.abs(mLeftDrive.getError()) < DRIVE.kGoalPosTolerance && Math.abs(mRightDrive.getError()) < DRIVE.kGoalPosTolerance;
  }

  public boolean isVisionFinished(double dist, double angle) {
    return Math.abs(dist - mPeriodicIO.rightPos) < DRIVE.kGoalPosTolerance && Math.abs(angle - mPeriodicIO.gyro_heading.getDegrees()) < 1.5;
  }

  public DriveControlState getDriveControlState() {
    return mDriveControlState;
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
   * @param path Robot Path
   * @param dist_tol Position Tolerance for Path Follower
   * @param ang_tol Robot Angle Tolerance for Path Follower (Degrees)
   */
  public synchronized void setDrivePath(Path path, double dist_tol, double ang_tol) {
    Logger.logMarker("Began Path: " + path.getName());
    double offset = lastAngle - Pathfinder.boundHalfDegrees(Pathfinder.r2d(path.getLeftWheelTrajectory().get(0).heading));
    for (Trajectory.Segment segment : path.getLeftWheelTrajectory().segments) {
      segment.heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(segment.heading) + offset);
    }
    for (Trajectory.Segment segment : path.getRightWheelTrajectory().segments) {
      segment.heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(segment.heading) + offset);
    }
    zero();
    pathFollower = new PathFollower(path, dist_tol, ang_tol);
    mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
  }

  /*
   * Called from Auto Action to check when the path finishes. Saves the last angle to use with the
   * next path and resets the Trajectory Status
   */
  public synchronized boolean isPathFinished() {
    if (pathFollower.getFinished()) {
      lastAngle = pathFollower.getEndHeading();
      mDriveControlState = DriveControlState.OPEN_LOOP;
      pathFollower = null;
      leftStatus = TrajectoryStatus.NEUTRAL;
      rightStatus = TrajectoryStatus.NEUTRAL;
      return true;
    }
    return false;
  }

  /**
   * Called from Looper during Path Following Gets a TrajectoryStatus containing output velocity and
   * Desired Trajectory Information for logging Inputs Position, Speed and Angle to Trajectory
   * Follower Creates a new Drive Signal that is then set as a velocity setpoint
   */
  private synchronized void updatePathFollower() {
    TrajectoryStatus leftUpdate = pathFollower.getLeftVelocity(mPeriodicIO.leftPos, mPeriodicIO.leftVel, -getHeadingDeg());
    TrajectoryStatus rightUpdate = pathFollower.getRightVelocity(mPeriodicIO.rightPos, mPeriodicIO.rightVel, -getHeadingDeg());
    leftStatus = leftUpdate;
    rightStatus = rightUpdate;
    setVelocity(new DriveSignal(leftUpdate.getOutput(), rightUpdate.getOutput()), new DriveSignal(leftUpdate.getArbFeed(), rightUpdate.getArbFeed()));
  }

  /**
   * Left is Positive Right is Negative (180 to -180)
   *
   * @return current fused heading from navX
   */
  public double getHeadingDeg() {
    return mPeriodicIO.gyro_heading.getDegrees();
  }

  public enum DriveControlState {
    OPEN_LOOP, MOTION_MAGIC, PIGEON_SERVO, VELOCITY_SETPOINT
  }
  private static class InstanceHolder {
    private static final Drive mInstance = new Drive();
  }
  public static class PeriodicIO {
    public double timestamp;
    // Inputs
    public double leftPos;
    public double rightPos;
    public double leftVel;
    public double rightVel;
    public double fusedHeading;
    public Rotation2d gyro_heading = Rotation2d.identity();
    // OUTPUTS
    public double left_demand;
    public double right_demand;
    public double left_feedforward;
    public double right_feedforward;
    public NeutralMode brake_mode;
  }
}
