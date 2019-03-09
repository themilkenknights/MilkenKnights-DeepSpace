package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.ActuateFrontSolenoids;
import frc.robot.auto.actions.ActuateRearSolenoids;
import frc.robot.auto.actions.CargoArmSetpoint;
import frc.robot.auto.actions.MotionMagicBlind;
import frc.robot.auto.actions.WaitAction;
import frc.robot.auto.actions.WaitForAngle;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Superstructure.ClimbState;

/**
 * Actuates Front Pnuematic Cylinders (Hatch Side) and waits until a certain pitch Drives forward,
 * retracts front, extends back actuators, and waits until close to level Drives forward again,
 * retracts rear actuators, waits 1 second, then drives forward.
 */
public class ClimbLevel2Mode extends AutoModeBase {

    @Override protected void routine() throws AutoModeEndedException {
        runAction(new CargoArmSetpoint(CargoArmState.INTAKE));
        runAction(new ActuateFrontSolenoids(ClimbState.LOWERED));
        runAction(new WaitForAngle(15, true));
        runAction(new MotionMagicBlind(13));
        runAction(new CargoArmSetpoint(CargoArmState.REVERSE_CARGOSHIP));
        runAction(new ActuateRearSolenoids(ClimbState.LOWERED));
        runAction(new ActuateFrontSolenoids(ClimbState.RETRACTED));
        runAction(new WaitForAngle(5.0, false));
        runAction(new MotionMagicBlind(12));
        runAction(new ActuateRearSolenoids(ClimbState.RETRACTED));
        runAction(new WaitAction(1.0));
        runAction(new MotionMagicBlind(14));
    }
}
