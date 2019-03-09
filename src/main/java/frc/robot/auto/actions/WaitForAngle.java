package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;

public class WaitForAngle implements Action {

	private double angle;
	private boolean mUp;

	public WaitForAngle(double angle, boolean mUp) {
		this.angle = angle;
		this.mUp = mUp;
	}

	@Override
	public boolean isFinished() {
		if (mUp) {
			return Drive.getInstance().getPitch() > angle;
		}
		return Drive.getInstance().getPitch() < angle;
	}

	@Override
	public void update() {
	}

	@Override
	public void done() {
	}

	@Override
	public void start() {
	}
}
