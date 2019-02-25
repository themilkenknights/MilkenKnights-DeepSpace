package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ActuateFrontSolenoids;
import frc.robot.auto.actions.ActuateRearSolenoids;
import frc.robot.auto.actions.MotionMagicBlind;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ClimbState;
import frc.robot.subsystems.Superstructure.RobotState;

public class ClimbLevel2Mode extends AutoModeBase {

    @Override protected void routine() throws AutoModeEndedException {
        Superstructure.getInstance().setRobotState(RobotState.AUTO_CLIMB);
        CargoArm.getInstance().setArmState(CargoArmState.INTAKE);
        runAction(new ActuateFrontSolenoids(ClimbState.LOWERED));
        runAction(new WaitAction(0.7));
        runAction(new MotionMagicBlind(13, 13));
        runAction(new WaitAction(0.6));
        CargoArm.getInstance().setArmState(CargoArmState.REVERSE_ROCKET_LEVEL_TWO);
        runAction(new ActuateRearSolenoids(ClimbState.LOWERED));
        runAction(new ActuateFrontSolenoids(ClimbState.RETRACTED));
        runAction(new WaitAction(0.9));
        runAction(new MotionMagicBlind(13, 13));
        runAction(new WaitAction(0.9));
        runAction(new ActuateRearSolenoids(ClimbState.RETRACTED));
        runAction(new WaitAction(0.5));
        runAction(new MotionMagicBlind(14, 14));
    }
}
