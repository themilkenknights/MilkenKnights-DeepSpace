package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicVision;

public class HatchIntakeVisionPigeon extends AutoModeBase {
    @Override protected void routine() throws AutoModeEndedException {
        runAction(new MotionMagicVision(false));
        // runAction(new MotionMagicHeadingVision());
       /* runAction(new MotionMagicBlind(-30.0));
        runAction(new SetHatchArmState(HatchMechanismState.STOWED));
        runAction(new CargoArmSetpoint(CargoArmState.INTAKE));
        runAction(new SetSuperstructure(RobotState.TELEOP_DRIVE)); */
    }
}
