package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CargoArmSetpoint;
import frc.robot.auto.actions.MotionMagicVisionPigeon;
import frc.robot.auto.actions.MotionMagicVisionPigeon.VisionServoGoal;
import frc.robot.auto.actions.OpenLoopDrive;
import frc.robot.subsystems.CargoArm.CargoArmState;

public class HatchOuttakeVisionPigeon extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new MotionMagicVisionPigeon(VisionServoGoal.PLACE_HATCH));
    runAction(new OpenLoopDrive(-0.25, -0.25, 0.5));
    runAction(new CargoArmSetpoint(CargoArmState.INTAKE));
  }
}
