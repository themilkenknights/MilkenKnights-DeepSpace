package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.MISC;
import frc.robot.auto.actions.VisionDrive;
import frc.robot.lib.drivers.CT;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.geometry.Kinematics;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.RobotState;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Twist2d;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTimer;
import frc.robot.lib.util.ReflectingCSVWriter;
import frc.robot.lib.util.Subsystem;
import frc.robot.lib.util.SynchronousPIDF;
import frc.robot.lib.util.TrajectoryStatus;
import frc.robot.lib.util.trajectory.Path;
import frc.robot.lib.util.trajectory.PathFollower;
import frc.robot.lib.vision.LimelightTarget;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class Drive extends Subsystem {
  private static SynchronousPIDF mVisionAssist = new SynchronousPIDF(Constants.DRIVE.kVisionP, Constants.DRIVE.kVisionI, Constants.DRIVE.kVisionD);
  private final MkTalon mLeftDrive, mRightDrive;
  public PeriodicIO mPeriodicIO;
  private DriveControlState mDriveControlState = DriveControlState.OPEN_LOOP;
  private Rotation2d mGyroOffset = Rotation2d.identity();
  private ReflectingCSVWriter<PeriodicIO> mCSVWriter;
  private NetworkTableEntry mState, mStatus, mFusedHeading, mGyroHeading, mAvgDist;
  private PigeonIMU mPigeon;
  private PathFollower pathFollower;
  private TrajectoryStatus leftStatus;
  private TrajectoryStatus rightStatus;
  private double lastAngle, lastDist, left_encoder_prev_distance_, right_encoder_prev_distance_;
  private boolean pathFinished, mLowered, mIsOnTarget;
  private VisionDrive.VisionGoal mGoal;
  private MkTimer placeCargoTimer = new MkTimer();
  private Rotation2d mTargetHeading = new Rotation2d();

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
    stateEstimator(timestamp);
    switch (mDriveControlState) {
      case OPEN_LOOP:
      case MOTION_MAGIC:
      case VISION_DRIVE:
        break;
      case TURN_IN_PLACE:
        updateTurnToHeading(timestamp);
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
   * Write setpoints to Talons and Victors left_demand and right_demand are always in Talon Native Units or Talon Native Units Per 100ms
   * left_feedforward and right_feedforward are in Percent Output [-1,1] and are added to the Talon SRX Closed Loop Output
   */
  @Override
  public synchronized void writePeriodicOutputs(double timestamp) {
    if (mDriveControlState == DriveControlState.OPEN_LOOP) {
      mLeftDrive.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, mPeriodicIO.brake_mode);
      mRightDrive.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, mPeriodicIO.brake_mode);
    } else if (mDriveControlState == DriveControlState.MOTION_MAGIC) {
      mLeftDrive.set(ControlMode.MotionMagic, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.left_feedforward,
          mPeriodicIO.brake_mode);
      mRightDrive.set(ControlMode.MotionMagic, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.right_feedforward,
          mPeriodicIO.brake_mode);
    } else if (mDriveControlState == DriveControlState.VELOCITY_SETPOINT) {
      mLeftDrive.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.left_feedforward,
          mPeriodicIO.brake_mode);
      mRightDrive.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.right_feedforward,
          mPeriodicIO.brake_mode);
    } else if (mDriveControlState == DriveControlState.VISION_DRIVE) {
      mLeftDrive.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, mPeriodicIO.brake_mode);
      mRightDrive.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, mPeriodicIO.brake_mode);
    } else if (mDriveControlState == DriveControlState.TURN_IN_PLACE) {
      mLeftDrive.set(ControlMode.MotionMagic, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.left_feedforward,
          mPeriodicIO.brake_mode);
      mRightDrive.set(ControlMode.MotionMagic, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, mPeriodicIO.right_feedforward,
          mPeriodicIO.brake_mode);
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
    mFusedHeading.setDouble(mPeriodicIO.fusedHeading);
    mAvgDist
        .setDouble((mLeftDrive.masterTalon.getSensorCollection().getQuadraturePosition() + mRightDrive.masterTalon.getSensorCollection()
            .getQuadraturePosition()) / 2.0);
    if (getHeading() != null) {
      mGyroHeading.setDouble(getHeadingDeg());
    }
    if (mCSVWriter != null && MISC.kDriveCSVLogging) {
      if (mDriveControlState == DriveControlState.VELOCITY_SETPOINT) {
        mPeriodicIO.leftDesiredPos = leftStatus.getSeg().position;
        mPeriodicIO.rightDesiredPos = rightStatus.getSeg().position;
        mPeriodicIO.desiredHeading = leftStatus.getSeg().heading;
        mPeriodicIO.headingError = leftStatus.getAngError();
        mPeriodicIO.leftVelError = leftStatus.getVelError();
        mPeriodicIO.leftPosError = leftStatus.getPosError();
        mPeriodicIO.rightVelError = rightStatus.getVelError();
        mPeriodicIO.rightPosError = rightStatus.getPosError();
        mPeriodicIO.desiredX = (leftStatus.getSeg().x + rightStatus.getSeg().x) / 2;
        mPeriodicIO.desiredY = (leftStatus.getSeg().y + rightStatus.getSeg().y) / 2;
      }
      mCSVWriter.add(mPeriodicIO);
      mCSVWriter.write();
    }
    mState.setString(mDriveControlState.toString());
    mStatus.setBoolean(driveStatus());
    SmartDashboard.putNumber("Main Target", mRightDrive.getTarget(0));
  }

  public void teleopInit(double timestamp) {
    zero();
    RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
  }

  /**
   * Clear mag encoder position and local distance counter and start logging if appropriate
   */
  @Override
  public void autonomousInit(double timestamp) {
    zero();
    RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
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
    zeroPigeon();
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
      clearOutput();

      mDriveControlState = DriveControlState.OPEN_LOOP;
    }
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.brake_mode = NeutralMode.Brake;
  }

  private synchronized void zero() {
    left_encoder_prev_distance_ = 0;
    right_encoder_prev_distance_ = 0;
    mLeftDrive.zeroEncoder();
    mRightDrive.zeroEncoder();
    setHeading(Rotation2d.identity());
    mPeriodicIO.leftPos = 0.0;
    mPeriodicIO.rightPos = 0.0;
    mPeriodicIO.leftVel = 0.0;
    mPeriodicIO.rightVel = 0.0;
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

  /**
   * @param dist Delta Position in inches
   * @param feed Left/Right arbitrary feed (Percent Output, [-1,1])
   */
  public synchronized void setMotionMagicPositionSetpoint(DriveSignal dist, DriveSignal feed) {
    if (mDriveControlState != DriveControlState.MOTION_MAGIC) {
      Logger.logMarker("Switching to Motion Magic");
      clearOutput();
      mDriveControlState = DriveControlState.MOTION_MAGIC;
    }
    mPeriodicIO.left_demand = MkMath.InchesToNativeUnits(dist.getLeft() + mPeriodicIO.leftPos);
    mPeriodicIO.right_demand = MkMath.InchesToNativeUnits(dist.getRight() + mPeriodicIO.rightPos);
    mPeriodicIO.left_feedforward = feed.getLeft();
    mPeriodicIO.right_feedforward = feed.getRight();
    mPeriodicIO.brake_mode = NeutralMode.Brake;
  }

  /**
   * Controls Drivetrain in Closed-loop velocity Mode Method sets Talons in Native Units per 100ms
   *
   * @param signal An object that contains left and right velocities (inches per sec)
   */
  private synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
    if (mDriveControlState != DriveControlState.VELOCITY_SETPOINT) {
      Logger.logMarker("Switching to Velocity");
      clearOutput();
      mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
    }
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = feedforward.getLeft();
    mPeriodicIO.right_feedforward = feedforward.getRight();
    mPeriodicIO.brake_mode = signal.getBrakeMode();
  }

  public synchronized void updateVisionDrive() {
    LimelightTarget target = Vision.getInstance().getLimelightTarget();
    switch (mGoal) {
      case INTAKE_HATCH:
        if (target.isValidTarget() && target.getDistance() < 55.0 && !mLowered) {
          HatchArm.getInstance().setHatchState(HatchArm.HatchState.INTAKE);
          mLowered = true;
        }
        break;
      case PLACE_HATCH:
        if (target.isValidTarget() && target.getDistance() < 26.0 && !mLowered) {
          HatchArm.getInstance().setHatchState(HatchArm.HatchState.PLACE);
          mLowered = true;
        }
        break;
      case PLACE_CARGO:
        if (target.getDistance() < 26.0 && !placeCargoTimer.hasBeenSet()) {
          placeCargoTimer.start(0.5);
        } else if (placeCargoTimer.isDone()) {
          CargoArm.getInstance().setIntakeRollers(Constants.CARGO_ARM.kCargoShipIntakeRollerOut);
        }
        break;
      default:
        Logger.logErrorWithTrace("Unknown Vision Goal");
        break;
    }

    double visionTurn = 0.0;
    double dist = target.getDistance();
    if (HatchArm.getInstance().getHatchSpearState() != HatchArm.HatchState.PLACE && ((lastDist - target.getDistance()) > -2.0)) {
      //double skew = ((target.getSkew() * Math.pow(dist, 3)) / 5.0e4) * 0.5 + ((Math.sin(Math.toRadians(target.getYaw())) * dist) / 2.0);
      visionTurn = mVisionAssist.calculate(target.getYaw());
    }
    double speed = 0.275;
    //double speed = -2.65 + 0.383917 * dist - 0.0196875 * Math.pow(dist,2) + 0.000483333 * Math.pow(dist,3) - 5.625e-6 * Math.pow(dist,4) + 2.5e-8 * Math.pow(dist,5);
    if (20.0 > dist) {
      speed = 0.15;
    } else if (70.0 < dist) {
      speed = 0.425;
    }
    //System.out.println(speed);
    mPeriodicIO.left_demand = speed - visionTurn;
    mPeriodicIO.right_demand = speed + visionTurn;
    lastDist = dist;
  }

  public boolean isCargoTimerDone() {
    return placeCargoTimer.isDone(1.0);
  }

  public synchronized void setVisionDrive(VisionDrive.VisionGoal mGoal) {
    this.mGoal = mGoal;
    mLowered = false;
    clearOutput();
    mDriveControlState = DriveControlState.VISION_DRIVE;
    placeCargoTimer.reset();
  }

  /**
   * @return Pitch from the Pigeon IMU [-90 to 90deg]
   */
  public double getPitch() {
    double[] arr = new double[3];
    mPigeon.getYawPitchRoll(arr);
    return arr[2];
  }

  private void clearOutput() {
    mPeriodicIO.left_demand = 0.0;
    mPeriodicIO.right_demand = 0.0;
    mPeriodicIO.left_feedforward = 0.0;
    mPeriodicIO.right_feedforward = 0.0;
  }

  public boolean isMotionMagicFinished() {
    return Math.abs(mPeriodicIO.rightPos - mPeriodicIO.right_demand) < 0.5;
  }

  public boolean isVisionFinished(double dist, double angle) {
    return Math.abs(dist - mPeriodicIO.rightPos) < DRIVE.kGoalPosTolerance && Math.abs(angle - mPeriodicIO.gyro_heading.getDegrees()) < 1.5;
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
    Superstructure.getInstance().setRobotState(Superstructure.RobotState.PATH_FOLLOWING);
    zero();
    Logger.logMarker("Began Path: " + path.getName());
    double offset = lastAngle - Pathfinder.boundHalfDegrees(Pathfinder.r2d(path.getLeftWheelTrajectory().get(0).heading));
    for (Trajectory.Segment segment : path.getLeftWheelTrajectory().segments) {
      segment.heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(segment.heading) + offset);
    }
    for (Trajectory.Segment segment : path.getRightWheelTrajectory().segments) {
      segment.heading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(segment.heading) + offset);
    }
    pathFollower = new PathFollower(path, dist_tol, ang_tol);
    mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
    pathFinished = false;
  }

  public synchronized void cancelPath() {
    pathFinished = true;
  }

  /*
   * Called from Auto Action to check when the path finishes. Saves the last angle to use with the
   * next path and resets the Trajectory Status
   */
  public synchronized boolean isPathFinished() {
    if (pathFinished) {
      lastAngle = pathFollower.getEndHeading();
      mDriveControlState = DriveControlState.OPEN_LOOP;
      pathFollower = null;
      leftStatus = TrajectoryStatus.NEUTRAL;
      rightStatus = TrajectoryStatus.NEUTRAL;
      return true;
    }
    return false;
  }

  public synchronized DriveControlState getDriveControlState() {
    return mDriveControlState;
  }

  /**
   * Called from Looper during Path Following Gets a TrajectoryStatus containing output velocity and Desired Trajectory Information for logging Inputs
   * Position, Speed and Angle to Trajectory Follower Creates a new Drive Signal that is then set as a velocity setpoint
   */
  private synchronized void updatePathFollower() {
    TrajectoryStatus leftUpdate = pathFollower.getLeftVelocity(mPeriodicIO.leftPos, mPeriodicIO.leftVel, getHeadingDeg());
    TrajectoryStatus rightUpdate = pathFollower.getRightVelocity(mPeriodicIO.rightPos, mPeriodicIO.rightVel, getHeadingDeg());
    leftStatus = leftUpdate;
    rightStatus = rightUpdate;
    double turn = 0.0;
  /*  LimelightTarget target = Vision.getInstance().getLimelightTarget();
   if (HatchArm.getInstance().getHatchSpearState() != HatchArm.HatchState.PLACE && target.isValidTarget() && pathFollower.getTimeLeft() < 0.2) {
      turn = mVisionAssist.calculate(target.getYaw());
    } */
    setVelocity(
        new DriveSignal(MkMath.InchesPerSecToUnitsPer100Ms(leftUpdate.getOutput()), MkMath.InchesPerSecToUnitsPer100Ms(rightUpdate.getOutput())),
        new DriveSignal(leftUpdate.getArbFeed() - turn, rightUpdate.getArbFeed() + turn));
    pathFinished = pathFollower.getFinished();
  }

  /**
   * Configures the drivebase to turn to a desired heading
   */
  public synchronized void setWantTurnToHeading(Rotation2d heading) {
    if (Math.abs(heading.inverse().rotateBy(mTargetHeading).getDegrees()) > 1E-3) {
      mTargetHeading = heading;
      mIsOnTarget = false;
    }
  }

  public synchronized boolean isTurnFinished() {
    return mIsOnTarget;
  }

  /**
   * Turn the robot to a target heading.
   *
   * Is called periodically when the robot is auto-aiming towards the boiler.
   */
  private synchronized void updateTurnToHeading(double timestamp) {
    final Rotation2d field_to_robot = RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation();

    // Figure out the rotation necessary to turn to face the goal.
    final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(mTargetHeading);

    // Check if we are on target
    final double kGoalPosTolerance = 0.75; // degrees
    final double kGoalVelTolerance = 5.0; // inches per second
    if (Math.abs(robot_to_target.getDegrees()) < kGoalPosTolerance
        && Math.abs(mPeriodicIO.leftVel) < kGoalVelTolerance
        && Math.abs(mPeriodicIO.rightVel) < kGoalVelTolerance) {
      // Use the current setpoint and base lock.
      mIsOnTarget = true;
      mPeriodicIO.left_demand = MkMath.InchesToNativeUnits(mPeriodicIO.leftPos);
      mPeriodicIO.right_demand = MkMath.InchesToNativeUnits(mPeriodicIO.rightPos);
      return;
    }

    Kinematics.DriveVelocity wheel_delta = Kinematics
        .inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
    mPeriodicIO.left_demand = MkMath.InchesToNativeUnits(wheel_delta.left + mPeriodicIO.leftPos);
    mPeriodicIO.right_demand = MkMath.InchesToNativeUnits(wheel_delta.right + mPeriodicIO.rightPos);
  }

  /**
   * Left is Positive Right is Negative (180 to -180)
   *
   * @return current fused heading from navX
   */
  private double getHeadingDeg() {
    return mPeriodicIO.gyro_heading.getDegrees();
  }

  public enum DriveControlState {
    OPEN_LOOP,
    MOTION_MAGIC,
    TURN_IN_PLACE,
    VISION_DRIVE,
    VELOCITY_SETPOINT
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
    public double desiredHeading;
    public double headingError;
    public double leftDesiredPos;
    public double leftPosError;
    public double leftVelError;
    public double rightDesiredPos;
    public double rightPosError;
    public double rightVelError;
    public double desiredX;
    public double desiredY;
  }
}
