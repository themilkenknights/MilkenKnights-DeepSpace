package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicVisionFeed;
import frc.robot.auto.actions.MotionMagicVisionPigeon;
import frc.robot.auto.actions.MotionMagicVisionPigeon.VisionServoGoal;

public class HatchIntakeVisionPigeon extends AutoModeBase {
    @Override protected void routine() throws AutoModeEndedException {
        runAction(new MotionMagicVisionPigeon(VisionServoGoal.INTAKE_HATCH));
        // runAction(new MotionMagicVisionPigeon());
       /* runAction(new MotionMagicBlind(-30.0));
        runAction(new SetHatchArmState(HatchMechanismState.STOWED));
        runAction(new CargoArmSetpoint(CargoArmState.INTAKE));
        runAction(new SetSuperstructure(RobotState.TELEOP_DRIVE)); */
    }
}
