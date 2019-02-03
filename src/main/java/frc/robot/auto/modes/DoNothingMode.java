package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.lib.util.Logger;

public class DoNothingMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		Logger.logMarker("Doing nothing");
	}
}
