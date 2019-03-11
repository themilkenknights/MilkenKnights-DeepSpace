package frc.robot.auto.modes;

import frc.robot.Constants.CARGO_ARM;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicVisionPigeon;
import frc.robot.auto.actions.MotionMagicVisionPigeon.VisionServoGoal;
import frc.robot.auto.actions.RollerAction;

public class VisionCargoOuttake extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new MotionMagicVisionPigeon(VisionServoGoal.PLACE_CARGO));
    runAction(new RollerAction(CARGO_ARM.kCargoShipIntakeRollerOut, 1.0));
  }
}
