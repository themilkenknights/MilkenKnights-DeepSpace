package frc.robot.auto.modes;

import frc.robot.Constants.CARGO_ARM;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicPixy;
import frc.robot.auto.actions.OpenLoopPixy;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.RollerAction;
import java.util.Arrays;

public class CargoVisionIntake extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new ParallelAction(
				Arrays.asList(new OpenLoopPixy(), new RollerAction(CARGO_ARM.INTAKE_IN_ROLLER_SPEED, 10.0))));
	}
}
