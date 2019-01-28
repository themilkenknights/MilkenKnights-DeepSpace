package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.VISION;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.TrackTarget;
import frc.robot.lib.drivers.MkGyro;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLocation;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Pose2dWithCurvature;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Twist2d;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.lib.trajectory.timing.TimedState;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.ReflectingCSVWriter;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.paths.DriveMotionPlanner;
import frc.robot.paths.Kinematics;
import frc.robot.paths.RobotState;

public class Drive extends Subsystem {

  private static AutoModeExecutor mAutoModeExecuter = null;
  private final MkTalon leftDrive, rightDrive;
  private final MkGyro navX;
  public PeriodicIO mPeriodicIO;
  public DriveControlState mDriveControlState;
  private DriveMotionPlanner mMotionPlanner;
  private Rotation2d mGyroOffset = Rotation2d.identity();
  private boolean mOverrideTrajectory, isVisionDone, mIsOnTarget = false;
  private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
  private double left_encoder_prev_distance_, right_encoder_prev_distance_, initial_vision_dist_, initial_vision_encoder_avg, turnAngle = 0.0;

  private Drive() {
    mDriveControlState = DriveControlState.OPEN_LOOP;
    mPeriodicIO = new PeriodicIO();
    leftDrive = new MkTalon(Constants.CAN.kDriveLeftMasterID, Constants.CAN.kDriveLeftSlaveID,
        TalonLocation.Left_Drive);
    rightDrive = new MkTalon(Constants.CAN.kDriveRightMasterID, Constants.CAN.kDriveRightSlaveID,
        TalonLocation.Right_Drive);
    leftDrive.resetEncoder();
    rightDrive.resetEncoder();
    navX = new MkGyro(Port.kMXP);
    leftDrive.masterTalon.setInverted(Constants.DRIVE.kLeftMasterInvert);
    leftDrive.slaveTalon.setInverted(Constants.DRIVE.kLeftSlaveInvert);
    leftDrive.masterTalon.setSensorPhase(Constants.DRIVE.kLeftSensorInvert);
    rightDrive.masterTalon.setInverted(Constants.DRIVE.KRightMasterInvert);
    rightDrive.slaveTalon.setInverted(Constants.DRIVE.kRightSlaveInvert);
    rightDrive.masterTalon.setSensorPhase(Constants.DRIVE.kRightSensorInvert);
    mMotionPlanner = new DriveMotionPlanner();
  }

  public static Drive getInstance() {
    return InstanceHolder.mInstance;
  }

  private synchronized void stateEstimator(double timestamp) {
    final double left_distance = mPeriodicIO.leftPos;
    final double right_distance = mPeriodicIO.rightPos;
    final double delta_left = left_distance - left_encoder_prev_distance_;
    final double delta_right = right_distance - right_encoder_prev_distance_;
    final Rotation2d gyro_angle = mPeriodicIO.gyro_heading;
    final Twist2d odometry_velocity = RobotState.getInstance()
        .generateOdometryFromSensors(delta_left, delta_right, gyro_angle);
    final Twist2d predicted_velocity = Kinematics
        .forwardKinematics(mPeriodicIO.leftVel, mPeriodicIO.leftVel);
    RobotState.getInstance().addObservations(timestamp, odometry_velocity, predicted_velocity);
    left_encoder_prev_distance_ = left_distance;
    right_encoder_prev_distance_ = right_distance;
  }

  private void updatePathFollower() {
    if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
      final double now = Timer.getFPGATimestamp();
      DriveMotionPlanner.Output output = mMotionPlanner
          .update(now, RobotState.getInstance().getFieldToVehicle(now));
      mPeriodicIO.error = mMotionPlanner.error();
      mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();
      if (!mOverrideTrajectory) {
        setVelocity(new DriveSignal(MkMath.radiansPerSecondToTicksPer100ms(output.left_velocity),
                MkMath.radiansPerSecondToTicksPer100ms(output.right_velocity)),
            new DriveSignal(output.left_feedforward_voltage / 12.0,
                output.right_feedforward_voltage / 12.0));
        mPeriodicIO.left_accel = MkMath.radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
        mPeriodicIO.right_accel =
            MkMath.radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
      } else {
        setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
        mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
      }
    } else {
      Logger.logError("Drive is not in path following state");
    }
  }

  private void updateVision() {
    double deltaAvg = Math.abs(initial_vision_encoder_avg - getAvgEncoderDist());
    if (leftDrive.masterTalon.getSensorCollection().isFwdLimitSwitchClosed()) {
      isVisionDone = true;
      setOpenLoop(DriveSignal.BRAKE);
      initial_vision_dist_ = 0;
      initial_vision_encoder_avg = 0;
    } else if ((deltaAvg / initial_vision_dist_) < 0.75) {
      LimelightTarget target = Vision.getInstance().getAverageTarget();
      double dist = MkMath.InchesToNativeUnits(VISION.visionDistMap.getInterpolated(new InterpolatingDouble(target.getArea())).value);
      double steering_adjust = DRIVE.kVisionTurnP * target.getXOffset();
      mPeriodicIO.left_demand = dist;
      mPeriodicIO.right_demand = dist;
      mPeriodicIO.left_feedforward = steering_adjust;
      mPeriodicIO.right_feedforward = -steering_adjust;
    } else {
      setOpenLoop(new DriveSignal(0.25, 0.25));
    }
  }

  private void updateTurnToHeading(double timestamp) {
    final Rotation2d field_to_robot = RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation();
    // Figure out the rotation necessary to turn to face the goal.
    final Rotation2d robot_to_target = field_to_robot.inverse().rotateBy(Rotation2d.fromDegrees(turnAngle));
    // Check if we are on target
    if (Math.abs(robot_to_target.getDegrees()) < Constants.DRIVE.kGoalPosTolerance
        && Math.abs(mPeriodicIO.leftVel) < Constants.DRIVE.kGoalVelTolerance && Math.abs(mPeriodicIO.rightVel) < Constants.DRIVE.kGoalVelTolerance) {
      // Use the current setpoint and base lock.
      mIsOnTarget = true;
      turnAngle = 0;
      updatePositionSetpoint(mPeriodicIO.leftPos, mPeriodicIO.rightPos);
      return;
    }
    Kinematics.DriveVelocity wheel_delta = Kinematics.inverseKinematics(new Twist2d(0, 0, robot_to_target.getRadians()));
    updatePositionSetpoint(wheel_delta.left + mPeriodicIO.leftPos, wheel_delta.right + mPeriodicIO.rightPos);
  }

  private double getAvgEncoderDist() {
    return (left_encoder_prev_distance_ + right_encoder_prev_distance_) / 2.0;
  }

  private void updatePositionSetpoint(double left_position_inches, double right_position_inches) {
    if (mDriveControlState == DriveControlState.TURN_IN_PLACE || mDriveControlState == DriveControlState.VISION_TRACKING) {
      mPeriodicIO.left_demand = left_position_inches;
      mPeriodicIO.right_demand = right_position_inches;
      mPeriodicIO.left_feedforward = 0.0;
      mPeriodicIO.right_feedforward = 0.0;
    } else {
      Logger.logError("Hit a bad position control state");
      setOpenLoop(DriveSignal.BRAKE);
    }
  }

  /*
  Check if the trajectory is finished
   */
  public boolean isDoneWithTrajectory() {
    if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
      return false;
    }
    return mMotionPlanner.isDone() || mOverrideTrajectory;
  }

  public boolean isTurnDone() {
    return mIsOnTarget;
  }

  /**
   * Controls Drivetrain in Closed-loop velocity Mode Method sets Talons in Native Units per 100ms
   *
   * @param signal An object that contains left and right velocities (inches per sec)
   */
  public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
    if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
      mDriveControlState = DriveControlState.PATH_FOLLOWING;
    }
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = feedforward.getLeft();
    mPeriodicIO.right_feedforward = feedforward.getRight();
  }

  /* Controls Drivetrain in PercentOutput Mode (without closed loop control) */
  public synchronized void setOpenLoop(DriveSignal signal) {
    if (mDriveControlState != DriveControlState.OPEN_LOOP) {
      Logger.logMarker("Switching to open loop");
      mDriveControlState = DriveControlState.OPEN_LOOP;
    }
    mPeriodicIO.left_demand = signal.getLeft();
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_feedforward = 0.0;
    mPeriodicIO.right_feedforward = 0.0;
  }

  public synchronized void startLogging() {
    if (mCSVWriter == null && Constants.LOG.kDriveCSVLogging) {
      mCSVWriter = new ReflectingCSVWriter<>("DRIVE-LOGS", PeriodicIO.class);
    }
  }

  public synchronized void stopLogging() {
    if (mCSVWriter != null) {
      mCSVWriter.flush();
      mCSVWriter = null;
    }
  }

  public boolean driveStatus() {
    return leftDrive.isEncoderConnected() && rightDrive.isEncoderConnected() && navX.isConnected();
  }

  /*
  Step 1: Read inputs from Talon and NavX
   */
  @Override
  public synchronized void readPeriodicInputs(double timestamp) {
    mPeriodicIO.timestamp = Timer.getFPGATimestamp();
    mPeriodicIO.leftPos = leftDrive.getPosition();
    mPeriodicIO.rightPos = rightDrive.getPosition();
    mPeriodicIO.leftVel = leftDrive.getSpeed();
    mPeriodicIO.rightVel = rightDrive.getSpeed();
    mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(navX.getFusedHeading()).rotateBy(mGyroOffset);
  }

  /*
  Step 3: Write setpoints to Talon
   */
  @Override
  public synchronized void writePeriodicOutputs(double timestamp) {
    if (mDriveControlState == DriveControlState.OPEN_LOOP) {
      leftDrive.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, NeutralMode.Brake);
      rightDrive.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, NeutralMode.Brake);
    } else if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
      leftDrive.set(ControlMode.Velocity, mPeriodicIO.left_demand, NeutralMode.Brake,
          mPeriodicIO.left_feedforward + Constants.DRIVE.kDriveD * mPeriodicIO.left_accel / 1023.0);
      rightDrive.set(ControlMode.Velocity, mPeriodicIO.right_demand, NeutralMode.Brake,
          mPeriodicIO.right_feedforward
              + Constants.DRIVE.kDriveD * mPeriodicIO.right_accel / 1023.0);
    } else {
      leftDrive.set(ControlMode.MotionMagic, mPeriodicIO.left_demand, NeutralMode.Brake, mPeriodicIO.left_feedforward);
      rightDrive.set(ControlMode.MotionMagic, mPeriodicIO.right_demand, NeutralMode.Brake, mPeriodicIO.right_feedforward);
    }
  }

  public void outputTelemetry() {
    leftDrive.updateSmartDash();
    rightDrive.updateSmartDash();
    SmartDashboard.putString("Drive State", mDriveControlState.toString());
    SmartDashboard.putBoolean("Drivetrain Status", driveStatus());
    SmartDashboard.putNumber("NavX Fused Heading", navX.getFusedHeading());
    if (mCSVWriter != null) {
      mCSVWriter.add(mPeriodicIO);
      mCSVWriter.write();
    }
  }

  @Override
  public void onStart(double timestamp) {
    synchronized (Drive.this) {
      startLogging();
    }
  }

  /**
   * Updated from mEnabledLoop in Robot.java Controls drivetrain during Path Following and Turn In Place and logs Drivetrain data in all modes
   *
   * @param timestamp In Seconds Since Code Start
   */
  @Override
  public void onLoop(double timestamp) {
    synchronized (Drive.this) {
      stateEstimator(timestamp);
      switch (mDriveControlState) {
        case OPEN_LOOP:
          break;
        case PATH_FOLLOWING:
          updatePathFollower();
          break;
        case VISION_TRACKING:
          updateVision();
          break;
        case TURN_IN_PLACE:
          updateTurnToHeading(timestamp);
          break;
        default:
          Logger.logError("Unexpected drive control state: " + mDriveControlState);
          break;
      }
    }
  }

  @Override
  public void onStop(double timestamp) {
    setOpenLoop(DriveSignal.BRAKE);
    stopLogging();
  }

  public boolean checkSystem() {
    boolean driveCheck = leftDrive.checkSystem() && rightDrive.checkSystem();
    if (!navX.isConnected()) {
      Logger.logError("FAILED - NAVX DISCONNECTED");
      driveCheck = false;
    } else {
      Logger.logMarker("NavX Connected");
    }
    if (driveCheck) {
      Logger.logMarker("Drive Test Success");
    }
    leftDrive.resetMasterConfig();
    leftDrive.resetSlaveConfig();
    rightDrive.resetMasterConfig();
    rightDrive.resetSlaveConfig();
    return driveCheck;
  }

  /*
  Zero heading by adding an offset
   */
  public synchronized void setHeading(Rotation2d heading) {
    Logger.logMarker("SET HEADING: " + heading.getDegrees());
    mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(navX.getFusedHeading()).inverse());
    Logger.logMarker("Gyro offset: " + mGyroOffset.getDegrees());
    mPeriodicIO.gyro_heading = heading;
  }

  /*
Pass the trajectory to the drive motion planner
 */
  public synchronized void setTrajectory(
      TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
    if (mMotionPlanner != null) {
      mOverrideTrajectory = false;
      mMotionPlanner.reset();
      mMotionPlanner.setTrajectory(trajectory);
      mDriveControlState = DriveControlState.PATH_FOLLOWING;
    }
  }

  public void startTurnInPlace(double angle) {
    mDriveControlState = DriveControlState.TURN_IN_PLACE;
    turnAngle = angle;
  }

  public void startVisionTracking() {
    LimelightTarget target = Vision.getInstance().getAverageTarget();
    if (target.isValidTarget()) {
      setOpenLoop(DriveSignal.BRAKE);
      mDriveControlState = DriveControlState.VISION_TRACKING;
      if (mAutoModeExecuter != null) {
        mAutoModeExecuter.stop();
      }
      mAutoModeExecuter = null;
      mAutoModeExecuter = new AutoModeExecutor();
      double dist = Constants.VISION.visionDistMap.getInterpolated(new InterpolatingDouble(target.getArea())).value;
      mAutoModeExecuter.setAutoMode(new TrackTarget());
      mAutoModeExecuter.start();
      initial_vision_dist_ = dist;
      updateVision();
    }
  }

  public boolean visionTrackingDone() {
    return isVisionDone;
  }

  /*
  Clear mag encoder position and local distance counter
   */
  public synchronized void zeroSensors() {
    navX.zeroYaw();
    leftDrive.masterTalon.setSelectedSensorPosition(0);
    rightDrive.masterTalon.setSelectedSensorPosition(0);
    left_encoder_prev_distance_ = 0;
    right_encoder_prev_distance_ = 0;
  }


  public enum DriveControlState {
    OPEN_LOOP, // open loop voltage control
    PATH_FOLLOWING, // velocity PID control
    VISION_TRACKING, //Vision tracking the Alliance Station, Rocket, or Cargo Ship Retroreflective Tape
    TURN_IN_PLACE //Rotate the robot without translating
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
    public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(
        Pose2dWithCurvature.identity());
  }
}
