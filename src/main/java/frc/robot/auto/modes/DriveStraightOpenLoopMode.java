package frc.robot.auto.modes;

import frc.robot.auto.actions.OpenLoopAction;
import frc.robot.util.auto.AutoModeBase;
import frc.robot.util.auto.AutoModeEndedException;
import frc.robot.util.logging.Log;

public class DriveStraightOpenLoopMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		Log.marker("Started Drive Straight Open Loop Auto");
		runAction(new OpenLoopAction(2, 0.4, false));
		runAction(new OpenLoopAction(1.5, 0.2, true));
	}
}
