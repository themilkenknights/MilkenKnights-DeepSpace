package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.drivers.MkGyro;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonPosition;
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
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.paths.DriveMotionPlanner;
import frc.robot.paths.Kinematics;
import frc.robot.paths.RobotState;

public class Drive extends Subsystem {
		//private static AutoModeExecutor mAutoModeExecuter = null;
		private final MkTalon leftDrive, rightDrive;
		private final MkGyro navX;
		public PeriodicIO mPeriodicIO;
		public DriveControlState mDriveControlState;
		private DriveMotionPlanner mMotionPlanner;
		private Rotation2d mGyroOffset = Rotation2d.identity();
		private boolean mOverrideTrajectory = false;
		//private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
		private double left_encoder_prev_distance_ = 0.0;
		private double right_encoder_prev_distance_ = 0.0;

		private Drive() {
				mDriveControlState = DriveControlState.OPEN_LOOP;
				mPeriodicIO = new PeriodicIO();
				leftDrive = new MkTalon(Constants.kDriveLeftMasterID, Constants.kDriveLeftSlaveID, TalonPosition.Left);
				rightDrive = new MkTalon(Constants.kDriveRightMasterID, Constants.kDriveRightSlaveID, TalonPosition.Right);
				leftDrive.resetEncoder();
				rightDrive.resetEncoder();
				navX = new MkGyro(Port.kMXP);
				leftDrive.masterTalon.setInverted(Constants.kLeftMasterInvert);
				leftDrive.slaveTalon.setInverted(Constants.kLeftSlaveInvert);
				leftDrive.masterTalon.setSensorPhase(Constants.kLeftSensorInvert);
				rightDrive.masterTalon.setInverted(Constants.KRightMasterInvert);
				rightDrive.slaveTalon.setInverted(Constants.kRightSlaveInvert);
				rightDrive.masterTalon.setSensorPhase(Constants.kRightSensorInvert);
				mMotionPlanner = new DriveMotionPlanner();
		}

		public static Drive getInstance() {
				return InstanceHolder.mInstance;
		}

		public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
				if (mMotionPlanner != null) {
						mOverrideTrajectory = false;
						mMotionPlanner.reset();
						mMotionPlanner.setTrajectory(trajectory);
						mDriveControlState = DriveControlState.PATH_FOLLOWING;
				}
		}

		public boolean isDoneWithTrajectory() {
				if (mMotionPlanner == null || mDriveControlState != DriveControlState.PATH_FOLLOWING) {
						return false;
				}
				return mMotionPlanner.isDone() || mOverrideTrajectory;
		}

		public synchronized void setHeading(Rotation2d heading) {
				System.out.println("SET HEADING: " + heading.getDegrees());
				mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(navX.getFusedHeading()).inverse());
				System.out.println("Gyro offset: " + mGyroOffset.getDegrees());
				mPeriodicIO.gyro_heading = heading;
		}

		public synchronized void zeroSensors() {
				navX.zeroYaw();
				leftDrive.masterTalon.setSelectedSensorPosition(0);
				rightDrive.masterTalon.setSelectedSensorPosition(0);
				left_encoder_prev_distance_ = 0;
				right_encoder_prev_distance_ = 0;
		}

		@Override
		public synchronized void readPeriodicInputs(double timestamp) {
				mPeriodicIO.timestamp = Timer.getFPGATimestamp();
				mPeriodicIO.leftPos = leftDrive.getPosition();
				mPeriodicIO.rightPos = rightDrive.getPosition();
				mPeriodicIO.leftVel = leftDrive.getSpeed();
				mPeriodicIO.rightVel = rightDrive.getSpeed();
				mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(navX.getFusedHeading()).rotateBy(mGyroOffset);
		}

		@Override
		public synchronized void writePeriodicOutputs(double timestamp) {
				if (mDriveControlState == DriveControlState.OPEN_LOOP) {
						leftDrive.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, NeutralMode.Brake);
						rightDrive.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, NeutralMode.Brake);
				} else {
						leftDrive.set(ControlMode.Velocity, mPeriodicIO.left_demand, NeutralMode.Brake,
								mPeriodicIO.left_feedforward + Constants.kDriveD * mPeriodicIO.left_accel / 1023.0);
						rightDrive.set(ControlMode.Velocity, mPeriodicIO.right_demand, NeutralMode.Brake,
								mPeriodicIO.right_feedforward + Constants.kDriveD * mPeriodicIO.right_accel / 1023.0);
				}
		}

		public void outputTelemetry() {
				leftDrive.updateSmartDash();
				rightDrive.updateSmartDash();
				SmartDashboard.putString("Drive State", mDriveControlState.toString());
				SmartDashboard.putBoolean("Drivetrain Status", leftDrive.isEncoderConnected() && rightDrive.isEncoderConnected());
				SmartDashboard.putNumber("NavX Fused Heading", navX.getFusedHeading());
				/*if (mCSVWriter != null) {
						mCSVWriter.add(mPeriodicIO);
						mCSVWriter.write();
				}*/
		}

		@Override
		public void onStart(double timestamp) {
				synchronized (Drive.this) {
						startLogging();
				}
		}

		/**
		 * Updated from mEnabledLoop in Robot.java Controls drivetrain during Path
		 * Following and Turn In Place and logs Drivetrain data in all modes
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
								default:
										System.out.println("Unexpected drive control state: " + mDriveControlState);
										break;
						}
				}
		}

		@Override
		public void onStop(double timestamp) {
				setOpenLoop(DriveSignal.BRAKE);
				stopLogging();
		}

		/* Controls Drivetrain in PercentOutput Mode (without closed loop control) */
		public synchronized void setOpenLoop(DriveSignal signal) {
				if (mDriveControlState != DriveControlState.OPEN_LOOP) {
						leftDrive.setBrakeMode();
						rightDrive.setBrakeMode();
						System.out.println("Switching to open loop");
						mDriveControlState = DriveControlState.OPEN_LOOP;
				}
				mPeriodicIO.left_demand = signal.getLeft();
				mPeriodicIO.right_demand = signal.getRight();
				mPeriodicIO.left_feedforward = 0.0;
				mPeriodicIO.right_feedforward = 0.0;
		}

		public synchronized void stopLogging() {
        /*if (mCSVWriter != null) {
            mCSVWriter.flush();
            mCSVWriter = null;
        }*/
		}

		public synchronized void stateEstimator(double timestamp) {
				final double left_distance = mPeriodicIO.leftPos;
				final double right_distance = mPeriodicIO.rightPos;
				final double delta_left = left_distance - left_encoder_prev_distance_;
				final double delta_right = right_distance - right_encoder_prev_distance_;
				final Rotation2d gyro_angle = mPeriodicIO.gyro_heading;
				final Twist2d odometry_velocity = RobotState.getInstance().generateOdometryFromSensors(delta_left, delta_right, gyro_angle);
				final Twist2d predicted_velocity = Kinematics.forwardKinematics(mPeriodicIO.leftVel, mPeriodicIO.leftVel);
				RobotState.getInstance().addObservations(timestamp, odometry_velocity, predicted_velocity);
				left_encoder_prev_distance_ = left_distance;
				right_encoder_prev_distance_ = right_distance;
		}

		private void updatePathFollower() {
				if (mDriveControlState == DriveControlState.PATH_FOLLOWING) {
						final double now = Timer.getFPGATimestamp();
						DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));
						mPeriodicIO.error = mMotionPlanner.error();
						mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();
						if (!mOverrideTrajectory) {
								setVelocity(new DriveSignal(MkMath.radiansPerSecondToTicksPer100ms(output.left_velocity),
												MkMath.radiansPerSecondToTicksPer100ms(output.right_velocity)),
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

		public void updateVision() {
				LimelightTarget target = Vision.getInstance().getAverageTarget();
				InterpolatingDouble dist = Constants.visionDistMap.getInterpolated(new InterpolatingDouble(target.getArea()));
				double steering_adjust = 0.08 * target.getXOffset();
				leftDrive.set(ControlMode.MotionMagic, dist.value, NeutralMode.Brake, steering_adjust);
				rightDrive.set(ControlMode.MotionMagic, dist.value, NeutralMode.Brake, -steering_adjust);
		}

		/**
		 * Controls Drivetrain in Closed-loop velocity Mode Method sets Talons in Native Units per 100ms
		 *
		 * @param signal An object that contains left and right velocities (inches per sec)
		 */
		public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
				if (mDriveControlState != DriveControlState.PATH_FOLLOWING) {
						leftDrive.setBrakeMode();
						rightDrive.setBrakeMode();
						mDriveControlState = DriveControlState.PATH_FOLLOWING;
				}
				mPeriodicIO.left_demand = signal.getLeft();
				mPeriodicIO.right_demand = signal.getRight();
				mPeriodicIO.left_feedforward = feedforward.getLeft();
				mPeriodicIO.right_feedforward = feedforward.getRight();
		}

		public synchronized void startLogging() {
				/*if (mCSVWriter == null) {
						mCSVWriter = new ReflectingCSVWriter<>("DRIVE-LOGS", PeriodicIO.class);
				}*/
		}

		public void checkSystem() {
				boolean check = true;
				leftDrive.resetEncoder();
				leftDrive.setCoastMode();
				leftDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
				leftDrive.masterTalon.set(ControlMode.PercentOutput, 1);
				Timer.delay(2.0);
				if (leftDrive.getPosition() < Constants.kMinTestPos || leftDrive.getSpeed() < Constants.kMinTestVel) {
						Logger.logMarker("FAILED - LEFT MASTER DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
						Logger.logMarker("Left Master Drive Test Failed - Vel: " + leftDrive.getSpeed() + " Pos: " + leftDrive.getPosition());
						check = false;
				} else {
						Logger.logMarker("Left Master Position: " + leftDrive.getPosition() + " Left Master Speed: " + leftDrive.getSpeed());
				}
				leftDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
				leftDrive.masterTalon.set(ControlMode.PercentOutput, 0);
				leftDrive.resetEncoder();
				Timer.delay(1.0);
				leftDrive.masterTalon.set(ControlMode.PercentOutput, 0);
				leftDrive.slaveTalon.set(ControlMode.PercentOutput, 1);
				Timer.delay(2.0);
				if (leftDrive.getPosition() < Constants.kMinTestPos || leftDrive.getSpeed() < Constants.kMinTestVel) {
						Logger.logMarker("FAILED - LEFT SLAVE DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
						Logger.logMarker("Left Slave Drive Test Failed - Vel: " + leftDrive.getSpeed() + " Pos: " + leftDrive.getPosition());
						check = false;
				} else {
						Logger.logMarker("Left Slave Position: " + leftDrive.getPosition() + " Left Slave Speed: " + leftDrive.getSpeed());
				}
				leftDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
				leftDrive.masterTalon.set(ControlMode.PercentOutput, 0);
				leftDrive.resetEncoder();
				Timer.delay(1.0);
				rightDrive.setCoastMode();
				rightDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
				rightDrive.masterTalon.set(ControlMode.PercentOutput, 1);
				Timer.delay(2.0);
				if (rightDrive.getPosition() < Constants.kMinTestPos || rightDrive.getSpeed() < Constants.kMinTestVel) {
						Logger.logMarker("FAILED - RIGHT MASTER DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
						Logger.logMarker("Right Drive Test Failed - Vel: " + leftDrive.getSpeed() + " Pos: " + leftDrive.getPosition());
						check = false;
				} else {
						Logger.logMarker("Right Master Position: " + rightDrive.getPosition() + " Right Master Speed: " + rightDrive.getSpeed());
				}
				rightDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
				rightDrive.masterTalon.set(ControlMode.PercentOutput, 0);
				rightDrive.resetEncoder();
				Timer.delay(1.0);
				rightDrive.masterTalon.set(ControlMode.PercentOutput, 0);
				rightDrive.slaveTalon.set(ControlMode.PercentOutput, 1);
				Timer.delay(2.0);
				if (rightDrive.getPosition() < Constants.kMinTestPos || rightDrive.getSpeed() < Constants.kMinTestVel) {
						Logger.logMarker("FAILED - RIGHT SLAVE DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
						Logger.logMarker("Right Drive Test Failed - Vel: " + rightDrive.getSpeed() + " Pos: " + rightDrive.getPosition());
						check = false;
				} else {
						Logger.logMarker("Right Slave Position: " + rightDrive.getPosition() + " Right Slave Speed: " + rightDrive.getSpeed());
				}
				rightDrive.masterTalon.set(ControlMode.PercentOutput, 0);
				rightDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
				if (!navX.isConnected()) {
						System.out.println("FAILED - NAVX DISCONNECTED");
						check = false;
				}
				if (check) {
						Logger.logMarker("Drive Test Success");
				}
				leftDrive.resetConfig();
				rightDrive.resetConfig();
		}

		public void targetPos() {
     /*   LimelightTarget target = Superstructure.getInstance().getTarget();
        if (target.isValidTarget()) {
           if (mAutoModeExecuter != null) {
                mAutoModeExecuter.stop();
            }
            mAutoModeExecuter = null;
            mAutoModeExecuter = new AutoModeExecutor();
            double dist = Constants.visionDistMap.getInterpolated(new InterpolatingDouble(target.getArea())).value;
            double angle = target.getXOffset();
            mAutoModeExecuter.setAutoMode(new TrackTarget(new Pose2d(new Translation2d(dist, 0.0), Rotation2d.fromDegrees(angle))));
            mAutoModeExecuter.start();
        } */
		}


		public enum DriveControlState {
				OPEN_LOOP, // open loop voltage control
				PATH_FOLLOWING, // velocity PID control
				VISION_TRACKING
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
				public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
		}
}
