package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicHeadingVision;

public class HatchOuttakeVisionPigeon extends AutoModeBase {
    @Override protected void routine() throws AutoModeEndedException {
        runAction(new MotionMagicHeadingVision());
        //  runAction(new OpenLoopDrive(-0.25, -0.25, 1.0));
        //     runAction(new CargoArmSetpoint(CargoArmState.INTAKE));
    }
}
