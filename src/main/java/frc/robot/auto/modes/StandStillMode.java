package frc.robot.auto.modes;

import frc.robot.util.auto.AutoModeBase;
import frc.robot.util.auto.AutoModeEndedException;
import frc.robot.util.logging.Log;

/**
 * Fallback for when all autonomous modes do not work, resulting in a robot
 * standstill
 */
public class StandStillMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {
        Log.marker("Starting Stand Still Mode... Done!");
    }
}
