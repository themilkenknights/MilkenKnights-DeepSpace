package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.OpenLoopDrive;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.util.Logger;

public class CrossAutoLineMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		Logger.logMarker("Running Cross auto line");
		runAction(new WaitAction(5.0));
		runAction(new OpenLoopDrive(-0.3, -0.3, 5.0));
	}
}
