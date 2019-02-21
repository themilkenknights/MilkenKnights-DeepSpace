package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ActuateFrontSolenoids;
import frc.robot.auto.actions.ActuateRearSolenoids;
import frc.robot.auto.actions.MotionMagicBlind;
import frc.robot.subsystems.Superstructure.ClimbState;

public class ClimbLevel2Mode extends AutoModeBase {

    @Override protected void routine() throws AutoModeEndedException {
        runAction(new ActuateFrontSolenoids(ClimbState.LOWERED));
        runAction(new MotionMagicBlind(-5, -5));
        runAction(new ActuateRearSolenoids(ClimbState.LOWERED));
        runAction(new ActuateFrontSolenoids(ClimbState.RETRACTED));
        runAction(new MotionMagicBlind(-10, -10));
        runAction(new ActuateRearSolenoids(ClimbState.RETRACTED));
        runAction(new MotionMagicBlind(-6, -6));
    }
}
