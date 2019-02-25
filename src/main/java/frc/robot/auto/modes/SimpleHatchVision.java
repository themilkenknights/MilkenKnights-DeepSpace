package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicBlind;
import frc.robot.auto.actions.MotionMagicVision;
import frc.robot.auto.actions.TurnInPlace;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;

public class SimpleHatchVision extends AutoModeBase {

    @Override protected void routine() throws AutoModeEndedException {
        runAction(new MotionMagicVision(true));
        runAction(new MotionMagicBlind(-20.0));
        runAction(new TurnInPlace(180.0));
        Superstructure.getInstance().setRobotState(RobotState.TELEOP_DRIVE);
    }
}
