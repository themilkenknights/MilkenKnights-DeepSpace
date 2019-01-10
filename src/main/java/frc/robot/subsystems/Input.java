package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.lib.structure.ILooper;
import frc.robot.lib.structure.Loop;

public class Input extends Subsystem {

	private final MkJoystick driverJoystick = new MkJoystick(0);
	private final MkJoystickButton changeDriveMode = driverJoystick.getButton(1, "Change Drive Mode");

	public Input() {

	}

	public static Input getInstance() {
		return InstanceHolder.mInstance;
	}

	@Override
	public void outputTelemetry() {

	}

	@Override
	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				synchronized (Input.this) {

				}
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Input.this) {
					if (Robot.mMatchState.equals(Robot.MatchState.TELEOP)) {
						updateDriveInput();
					}
				}
			}

			@Override
			public void onStop(double timestamp) {
			}
		});
	}

	private void updateDriveInput() {
		double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
		double turn = (-driverJoystick.getRawAxis(0));
		Drive.getInstance().setOpenLoop(DriveHelper.cheesyDrive(forward, turn, true));
	}

	private static class InstanceHolder {

		private static final Input mInstance = new Input();
	}

}
