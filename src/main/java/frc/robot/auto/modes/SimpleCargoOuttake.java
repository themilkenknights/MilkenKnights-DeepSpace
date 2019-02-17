package frc.robot.auto.modes;

import frc.robot.Constants.CARGO_ARM;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicBlind;
import frc.robot.auto.actions.MotionMagicVision;
import frc.robot.auto.actions.RollerAction;
import frc.robot.auto.actions.TurnInPlace;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;

public class SimpleCargoOuttake extends AutoModeBase {

	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new MotionMagicVision(false));
		runAction(new RollerAction(CARGO_ARM.ROCKET_LEVEL_ONE_INTAKE_OUT_ROLLER_SPEED, 1.0));
		runAction(new MotionMagicBlind(-20.0, -20.0));
		runAction(new TurnInPlace(180.0));
		Superstructure.getInstance().setRobotState(RobotState.TELEOP_DRIVE);
	}
}
