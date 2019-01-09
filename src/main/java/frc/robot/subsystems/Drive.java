package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE;
import frc.robot.ControlState;
import frc.robot.MkMath;
import frc.robot.RobotState;
import frc.robot.lib.drivers.LimeLight;
import frc.robot.lib.drivers.MkGyro;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonPosition;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Pose2dWithCurvature;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.structure.Loop;
import frc.robot.lib.structure.Looper;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.lib.trajectory.timing.TimedState;
import frc.robot.lib.util.CrashTracker;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.ReflectingCSVWriter;
import frc.robot.planners.DriveMotionPlanner;

public class Drive extends Subsystem {

	private final MkTalon leftDrive, rightDrive;
	private final MkGyro navX;
	private DriveMotionPlanner mMotionPlanner;
	private Rotation2d mGyroOffset = Rotation2d.identity();
	private boolean mOverrideTrajectory = false;
	private PeriodicIO mPeriodicIO;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private DriveControlState mDriveControlState;
	private LimeLight limeLight;

	private Drive() {
		mPeriodicIO = new PeriodicIO();

		leftDrive = new MkTalon(DRIVE.LEFT_MASTER_ID, DRIVE.LEFT_SLAVE_ID, TalonPosition.Left);
		rightDrive = new MkTalon(DRIVE.RIGHT_MASTER_ID, DRIVE.RIGHT_SLAVE_ID, TalonPosition.Right);
		leftDrive.setPIDF();
		rightDrive.setPIDF();
		leftDrive.resetEncoder();
		rightDrive.resetEncoder();
		navX = new MkGyro(Port.kMXP);

		leftDrive.masterTalon.setInverted(DRIVE.LEFT_MASTER_INVERT);
		leftDrive.slaveTalon.setInverted(DRIVE.LEFT_SLAVE_INVERT);
		leftDrive.masterTalon.setSensorPhase(DRIVE.LEFT_INVERT_SENSOR);

		rightDrive.masterTalon.setInverted(DRIVE.RIGHT_MASTER_INVERT);
		rightDrive.slaveTalon.setInverted(DRIVE.RIGHT_SLAVE_INVERT);
		rightDrive.masterTalon.setSensorPhase(DRIVE.RIGHT_INVERT_SENSOR);
		limeLight = new LimeLight();
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


	public void outputTelemetry() {
		leftDrive.updateSmartDash();
		rightDrive.updateSmartDash();
		SmartDashboard.putString("Drive State", ControlState.mDriveControlState.toString());
		SmartDashboard.putBoolean("Drivetrain Status", leftDrive.isEncoderConnected() && rightDrive.isEncoderConnected());

		if (mCSVWriter != null) {
			mCSVWriter.write();
		}
	}

	public synchronized void setHeading(Rotation2d heading) {
		System.out.println("SET HEADING: " + heading.getDegrees());

		mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(navX.getFusedHeading()).inverse());
		System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

		mPeriodicIO.gyro_heading = heading;
	}


	/* Controls Drivetrain in PercentOutput Mode (without closed loop control) */
	public synchronized void setOpenLoop(DriveSignal signal) {
		if (mDriveControlState != DriveControlState.OPEN_LOOP) {
			leftDrive.setBrakeMode();
			rightDrive.setBrakeMode();
			System.out.println("Switching to open loop");
			System.out.println(signal);
			mDriveControlState = DriveControlState.OPEN_LOOP;
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = 0.0;
		mPeriodicIO.right_feedforward = 0.0;
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
			DriverStation.reportError("Drive is not in path following state", false);
		}
	}

	@Override
	public synchronized void readPeriodicInputs() {
		mPeriodicIO.leftPos = leftDrive.getPosition();
		mPeriodicIO.rightPos = rightDrive.getPosition();
		mPeriodicIO.leftVel = leftDrive.getSpeed();
		mPeriodicIO.rightVel = rightDrive.getSpeed();
		mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(navX.getFusedHeading()).rotateBy(mGyroOffset);
		if (mCSVWriter != null) {
			mCSVWriter.add(mPeriodicIO);
		}
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mDriveControlState == DriveControlState.OPEN_LOOP) {
			leftDrive.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, true);
			leftDrive.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, true);
		} else {
			leftDrive.set(ControlMode.Velocity, mPeriodicIO.left_demand, true,
					mPeriodicIO.left_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.left_accel / 1023.0);
			rightDrive.set(ControlMode.Velocity, mPeriodicIO.right_demand, true,
					mPeriodicIO.right_feedforward + Constants.kDriveLowGearVelocityKd * mPeriodicIO.right_accel / 1023.0);
		}
	}

	@Override
	public void checkSystem() {
		boolean check = true;
		leftDrive.resetEncoder();
		leftDrive.setCoastMode();
		leftDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
		leftDrive.masterTalon.set(ControlMode.PercentOutput, 1);
		Timer.delay(2.0);
		if (leftDrive.getPosition() < Constants.DRIVE.MIN_TEST_POS || leftDrive.getSpeed() < Constants.DRIVE.MIN_TEST_VEL) {
			CrashTracker.logMarker("FAILED - LEFT MASTER DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
			CrashTracker.logMarker("Left Master Drive Test Failed - Vel: " + leftDrive.getSpeed() + " Pos: " + leftDrive.getPosition());
			check = false;
		} else {
			CrashTracker.logMarker("Left Master Position: " + leftDrive.getPosition() + " Left Master Speed: " + leftDrive.getSpeed());
		}

		leftDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
		leftDrive.masterTalon.set(ControlMode.PercentOutput, 0);
		leftDrive.resetEncoder();
		Timer.delay(1.0);

		leftDrive.masterTalon.set(ControlMode.PercentOutput, 0);
		leftDrive.slaveTalon.set(ControlMode.PercentOutput, 1);
		Timer.delay(2.0);
		if (leftDrive.getPosition() < Constants.DRIVE.MIN_TEST_POS || leftDrive.getSpeed() < Constants.DRIVE.MIN_TEST_VEL) {
			CrashTracker.logMarker("FAILED - LEFT SLAVE DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
			CrashTracker.logMarker("Left Slave Drive Test Failed - Vel: " + leftDrive.getSpeed() + " Pos: " + leftDrive.getPosition());
			check = false;
		} else {
			CrashTracker.logMarker("Left Slave Position: " + leftDrive.getPosition() + " Left Slave Speed: " + leftDrive.getSpeed());
		}
		leftDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
		leftDrive.masterTalon.set(ControlMode.PercentOutput, 0);
		leftDrive.resetEncoder();
		Timer.delay(1.0);

		rightDrive.setCoastMode();
		rightDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
		rightDrive.masterTalon.set(ControlMode.PercentOutput, 1);
		Timer.delay(2.0);
		if (rightDrive.getPosition() < Constants.DRIVE.MIN_TEST_POS || rightDrive.getSpeed() < Constants.DRIVE.MIN_TEST_VEL) {
			CrashTracker.logMarker("FAILED - RIGHT MASTER DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
			CrashTracker.logMarker("Right Drive Test Failed - Vel: " + leftDrive.getSpeed() + " Pos: " + leftDrive.getPosition());
			check = false;
		} else {
			CrashTracker.logMarker("Right Master Position: " + rightDrive.getPosition() + " Right Master Speed: " + rightDrive.getSpeed());
		}

		rightDrive.slaveTalon.set(ControlMode.PercentOutput, 0);
		rightDrive.masterTalon.set(ControlMode.PercentOutput, 0);
		rightDrive.resetEncoder();
		Timer.delay(1.0);

		rightDrive.masterTalon.set(ControlMode.PercentOutput, 0);
		rightDrive.slaveTalon.set(ControlMode.PercentOutput, 1);
		Timer.delay(2.0);
		if (rightDrive.getPosition() < Constants.DRIVE.MIN_TEST_POS || rightDrive.getSpeed() < Constants.DRIVE.MIN_TEST_VEL) {
			CrashTracker.logMarker("FAILED - RIGHT SLAVE DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
			CrashTracker.logMarker("Right Drive Test Failed - Vel: " + rightDrive.getSpeed() + " Pos: " + rightDrive.getPosition());
			check = false;
		} else {
			CrashTracker.logMarker("Right Slave Position: " + rightDrive.getPosition() + " Right Slave Speed: " + rightDrive.getSpeed());
		}
		rightDrive.masterTalon.set(ControlMode.PercentOutput, 0);
		rightDrive.slaveTalon.set(ControlMode.PercentOutput, 0);

		if (!navX.isConnected()) {
			System.out.println("FAILED - NAVX DISCONNECTED");
			check = false;
		}

		if (check) {
			CrashTracker.logMarker("Drive Test Success");
		}

		leftDrive.resetConfig();
		rightDrive.resetConfig();
	}

	@Override
	public void stop() {

	}


	public void registerEnabledLoops(Looper enabledLooper) {
		Loop mLoop = new Loop() {

			@Override
			public void onStart(double timestamp) {
				synchronized (Drive.this) {
					leftDrive.resetEncoder();
					rightDrive.resetEncoder();
					navX.zeroYaw();
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
					switch (mDriveControlState) {
						case OPEN_LOOP:
							break;
						case PATH_FOLLOWING:
							updatePathFollower();
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
			}
		};
		enabledLooper.register(mLoop);
	}

	public double getYaw() {
		return navX.getYaw();
	}

	public void updateVision() {
		InterpolatingDouble dist = Constants.visionDistMap.getInterpolated(new InterpolatingDouble(limeLight.getTargetArea()));
		double steering_adjust = 0.08 * limeLight.getX();
		leftDrive.set(ControlMode.MotionMagic, dist.value, true, steering_adjust);
		rightDrive.set(ControlMode.MotionMagic, dist.value, true, -steering_adjust);
	}

	public double getLeftVelocityNativeUnits() {
		return MkMath.InchesPerSecToUnitsPer100Ms(mPeriodicIO.leftVel);
	}

	public double getRightVelocityNativeUnits() {
		return MkMath.InchesPerSecToUnitsPer100Ms(mPeriodicIO.rightVel);
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
