package frc.robot.auto.actions;

import frc.robot.lib.math.MkMath;
import frc.robot.subsystems.Drive;

public class TurnInPlace implements Action {

	private double angleDelta;

	public TurnInPlace(double angleDelta) {
		this.angleDelta = angleDelta;
	}

	@Override
	public boolean isFinished() {
		return Drive.getInstance().isTurnDone();
	}

	@Override
	public void update() {
		Drive.getInstance().updateTurnToHeading();
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		double mGoalTurnAngle = MkMath.normalAbsoluteAngleDegrees(Drive.getInstance().getFusedHeading() + angleDelta);
		Drive.getInstance().setTurnInPlaceHeading(mGoalTurnAngle);
	}
}
