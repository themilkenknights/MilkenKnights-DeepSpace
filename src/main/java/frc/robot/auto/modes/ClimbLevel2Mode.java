package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ActuateFrontSolenoids;
import frc.robot.auto.actions.ActuateRearSolenoids;
import frc.robot.auto.actions.CargoArmSetpoint;
import frc.robot.auto.actions.MotionMagicBlind;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.auto.actions.WaitAction;
import frc.robot.auto.actions.WaitForAngle;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Superstructure.ClimbState;
import frc.robot.subsystems.Superstructure.RobotState;

public class ClimbLevel2Mode extends AutoModeBase {

    @Override protected void routine() throws AutoModeEndedException {
        runAction(new CargoArmSetpoint(CargoArmState.INTAKE));
        runAction(new ActuateFrontSolenoids(ClimbState.LOWERED));
        runAction(new WaitForAngle(25, true));
        runAction(new MotionMagicBlind(13));
        runAction(new CargoArmSetpoint(CargoArmState.REVERSE_CARGOSHIP));
        runAction(new ActuateRearSolenoids(ClimbState.LOWERED));
        runAction(new ActuateFrontSolenoids(ClimbState.RETRACTED));
        runAction(new WaitForAngle(5, false));
        runAction(new MotionMagicBlind(13));
        runAction(new ActuateRearSolenoids(ClimbState.RETRACTED));
        runAction(new WaitAction(1.0));
        runAction(new MotionMagicBlind(14));
    }
}
