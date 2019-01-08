package frc.robot.subsystems;

import frc.robot.RobotState;
import frc.robot.RobotState.DriveControlState;
import frc.robot.util.drivers.MkJoystick;
import frc.robot.util.drivers.MkJoystickButton;
import frc.robot.util.math.DriveHelper;
import frc.robot.util.state.DriveSignal;
import frc.robot.util.structure.Subsystem;
import frc.robot.util.structure.loops.Loop;
import frc.robot.util.structure.loops.Looper;

public class Input extends Subsystem {

	private final MkJoystick driverJoystick = new MkJoystick(0);
	private final MkJoystickButton changeDriveMode = driverJoystick.getButton(1, "Change Drive Mode");

	public Input() {

	}

	public static Input getInstance() {
		return InstanceHolder.mInstance;
	}

	@Override
	public void outputToSmartDashboard() {

	}

	@Override
	public void checkSystem() {

	}

	@Override
	public void registerEnabledLoops(Looper enabledLooper) {
		Loop mLoop = new Loop() {

			@Override
			public void onStart(double timestamp) {
				synchronized (Input.this) {

				}
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Input.this) {
					if (RobotState.mMatchState.equals(RobotState.MatchState.TELEOP)) {
						updateDriveInput();
					}
				}
			}

			@Override
			public void onStop(double timestamp) {
			}
		};
		enabledLooper.register(mLoop);
	}

	private void updateDriveInput() {
		if (changeDriveMode.isPressed()) {
			Drive.getInstance().configVelocityControl();
			RobotState.mDriveControlState =
					RobotState.mDriveControlState.equals(DriveControlState.OPEN_LOOP) ? DriveControlState.VELOCITY_SETPOINT
							: DriveControlState.OPEN_LOOP;
		}
		double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
		double turn = (-driverJoystick.getRawAxis(0));
		DriveSignal sig = DriveHelper.cheesyDrive(forward, turn, true);
		if (RobotState.mDriveControlState == DriveControlState.VELOCITY_SETPOINT) {
			Drive.getInstance().setVelocitySetpoint(sig, 0, 0);
		} else if (RobotState.mDriveControlState == DriveControlState.OPEN_LOOP) {
			Drive.getInstance().setOpenLoop(sig);
		}

	}

	private static class InstanceHolder {

		private static final Input mInstance = new Input();
	}

}
