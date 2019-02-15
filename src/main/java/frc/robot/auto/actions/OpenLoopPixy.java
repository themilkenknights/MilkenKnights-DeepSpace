package frc.robot.auto.actions;

import frc.robot.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class OpenLoopPixy implements Action {


	@Override
	public boolean isFinished() {
		return Vision.mPixy.isCargoIntaked();
	}

	@Override
	public void update() {
		double turn = (157.5 - Vision.mPixy.getX()) * -0.001;
		Drive.getInstance().setOpenLoop(new DriveSignal(-0.3 + turn, -0.3 - turn));
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {

	}
}
