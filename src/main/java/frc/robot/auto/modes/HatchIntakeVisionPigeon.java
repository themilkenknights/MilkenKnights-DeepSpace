package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CargoArmSetpoint;
import frc.robot.auto.actions.MotionMagicBlind;
import frc.robot.auto.actions.MotionMagicHeadingVision;
import frc.robot.auto.actions.MotionMagicVision;
import frc.robot.auto.actions.SetHatchArmState;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import frc.robot.subsystems.Superstructure.RobotState;

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
