package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ActuateFrontSolenoids;
import frc.robot.auto.actions.ActuateRearSolenoids;
import frc.robot.auto.actions.CargoArmSetpoint;
import frc.robot.auto.actions.MotionMagicBlind;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Superstructure.ClimbState;
import frc.robot.subsystems.Superstructure.RobotState;

public class ClimbLevel2Mode extends AutoModeBase {

    @Override protected void routine() throws AutoModeEndedException {
        runAction(new CargoArmSetpoint(CargoArmState.INTAKE));
        runAction(new ActuateFrontSolenoids(ClimbState.LOWERED));
        runAction(new WaitAction(0.7));
        runAction(new MotionMagicBlind(13));
        runAction(new WaitAction(0.6));
        runAction(new CargoArmSetpoint(CargoArmState.REVERSE_CARGOSHIP));
        runAction(new ActuateRearSolenoids(ClimbState.LOWERED));
        runAction(new ActuateFrontSolenoids(ClimbState.RETRACTED));
        runAction(new WaitAction(0.9));
        runAction(new MotionMagicBlind(13));
        runAction(new WaitAction(0.9));
        runAction(new ActuateRearSolenoids(ClimbState.RETRACTED));
        runAction(new WaitAction(0.5));
        runAction(new MotionMagicBlind(14));
        runAction(new SetSuperstructure(RobotState.TELEOP_DRIVE));
    }
}
