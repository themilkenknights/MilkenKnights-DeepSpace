package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicBlind;

public class ClimbLevel2Mode extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new MotionMagicBlind(-5, -5));
	}
}
