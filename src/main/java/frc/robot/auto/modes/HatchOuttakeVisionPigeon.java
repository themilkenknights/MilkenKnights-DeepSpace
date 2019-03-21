package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DrivePathVision;
import frc.robot.auto.actions.MotionMagicVisionPigeon;
import frc.robot.auto.actions.MotionMagicVisionPigeon.VisionServoGoal;

public class HatchOuttakeVisionPigeon extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new DrivePathVision());
    //runAction(new MotionMagicVisionPigeon(VisionServoGoal.PLACE_HATCH));
    // runAction(new OpenLoopDrive(-0.25, -0.25, 0.5));
    // runAction(new CargoArmSetpoint(CargoArmState.REVERSE_CARGOSHIP));
    // runAction(new SetHatchArmState(HatchState.STOW));
  }
}
