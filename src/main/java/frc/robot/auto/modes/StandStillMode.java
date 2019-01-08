package frc.robot.auto.modes;

import frc.robot.util.auto.AutoModeBase;
import frc.robot.util.auto.AutoModeEndedException;
import frc.robot.util.logging.CrashTracker;

/**
 * Fallback for when all autonomous modes do not work, resulting in a robot standstill
 */
public class StandStillMode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		CrashTracker.logMarker("Starting Stand Still Mode... Done!");
	}
}
