package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CargoArmSetpoint;
import frc.robot.auto.actions.MotionMagicBlind;
import frc.robot.auto.actions.MotionMagicHeadingVision;
import frc.robot.auto.actions.OpenLoopDrive;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Superstructure.RobotState;

public class HatchOuttakeVisionPigeon extends AutoModeBase {
    @Override protected void routine() throws AutoModeEndedException {
        runAction(new MotionMagicHeadingVision());
        runAction(new OpenLoopDrive(-0.3, -0.3, 1.5));
        //runAction(new MotionMagicBlind(-10.0));
        //runAction(new CargoArmSetpoint(CargoArmState.INTAKE));
        //runAction(new SetSuperstructure(RobotState.TELEOP_DRIVE));
    }
}
