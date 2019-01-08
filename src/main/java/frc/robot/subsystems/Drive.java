package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE;
import frc.robot.RobotState;
import frc.robot.RobotState.DriveControlState;
import frc.robot.RobotState.MatchState;
import frc.robot.util.drivers.LimeLight;
import frc.robot.util.drivers.MkGyro;
import frc.robot.util.drivers.MkTalon;
import frc.robot.util.drivers.MkTalon.TalonPosition;
import frc.robot.util.logging.CrashTracker;
import frc.robot.util.logging.ReflectingCSVWriter;
import frc.robot.util.math.InterpolatingDouble;
import frc.robot.util.state.DriveSignal;
import frc.robot.util.structure.Subsystem;
import frc.robot.util.structure.loops.Loop;
import frc.robot.util.structure.loops.Looper;

public class Drive extends Subsystem {

	private final MkTalon leftDrive, rightDrive;
	private final MkGyro navX;
	private DriveSignal currentSetpoint;
	private DriveDebug mDebug;
	private ReflectingCSVWriter<DriveDebug> mCSVWriter = null;
	private LimeLight limeLight;

	private Drive() {
		mDebug = new DriveDebug();
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
		currentSetpoint = DriveSignal.BRAKE;
		limeLight = new LimeLight();
	}

	public static Drive getInstance() {
		return InstanceHolder.mInstance;
	}

	@Override
	public void outputToSmartDashboard() {
		leftDrive.updateSmartDash();
		rightDrive.updateSmartDash();
		SmartDashboard.putString("Drive State", RobotState.mDriveControlState.toString());
		SmartDashboard.putBoolean("Drivetrain Status", leftDrive.isEncoderConnected() && rightDrive.isEncoderConnected());
	}

	/* Controls Drivetrain in PercentOutput Mode (without closed loop control) */
	public synchronized void setOpenLoop(DriveSignal signal) {
		RobotState.mDriveControlState = DriveControlState.OPEN_LOOP;
		SmartDashboard.putNumber("Left Drive Sig", signal.getLeft());
		leftDrive.set(ControlMode.PercentOutput, signal.getLeft(), signal.getBrakeMode());
		rightDrive.set(ControlMode.PercentOutput, signal.getRight(), signal.getBrakeMode());
		currentSetpoint = signal;
	}

	/**
	 * Controls Drivetrain in Closed-loop velocity Mode Method sets Talons in Native Units per 100ms
	 *
	 * @param signal An object that contains left and right velocities (inches per sec)
	 */

	public synchronized void setVelocitySetpoint(DriveSignal signal, double leftFeed, double rightFeed) {

		RobotState.mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
		leftDrive.set(ControlMode.Velocity, signal.getLeftNativeVel(), signal.getBrakeMode());
		rightDrive.set(ControlMode.Velocity, signal.getRightNativeVel(), signal.getBrakeMode());
		currentSetpoint = signal;
	}

	@Override
	public void slowUpdate(double timestamp) {
		if (mCSVWriter != null && RobotState.mMatchState != MatchState.DISABLED) {
			mDebug.leftOutput = currentSetpoint.getLeft();
			mDebug.rightOutput = currentSetpoint.getRight();
			mCSVWriter.add(mDebug);
			mCSVWriter.write();
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
	public void registerEnabledLoops(Looper enabledLooper) {
		Loop mLoop = new Loop() {

			@Override
			public void onStart(double timestamp) {
				synchronized (Drive.this) {
					leftDrive.resetEncoder();
					rightDrive.resetEncoder();
					navX.zeroYaw();
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
					switch (RobotState.mDriveControlState) {
						case OPEN_LOOP:
							return;
						case VELOCITY_SETPOINT:
							return;
						case VISION_TRACKING:
							updateVision();
							return;
						default:
							CrashTracker.logMarker("Unexpected drive control state: " + RobotState.mDriveControlState);
							break;
					}
				}
			}

			@Override
			public void onStop(double timestamp) {
				setOpenLoop(DriveSignal.BRAKE);
				stopLogging();
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

	/*
	 * Change Talon PID Constants to reduce oscillation during teleop driving
	 */
	public void configVelocityControl() {
		leftDrive.configTeleopVelocity();
		rightDrive.configTeleopVelocity();
	}

	public boolean gyroConnected() {
		return navX.isConnected();
	}

	public boolean isEncodersConnected() {
		return leftDrive.isEncoderConnected() && rightDrive.isEncoderConnected();
	}

	public synchronized void startLogging() {
		if (mCSVWriter == null) {
			mCSVWriter = new ReflectingCSVWriter<>("DRIVE-LOGS", DriveDebug.class);
		}
	}

	public synchronized void stopLogging() {
		if (mCSVWriter != null) {
			mCSVWriter.flush();
			mCSVWriter = null;
		}
	}

	private static class InstanceHolder {

		private static final Drive mInstance = new Drive();
	}

	public static class DriveDebug {

		public double leftOutput;
		public double rightOutput;

	}

}
