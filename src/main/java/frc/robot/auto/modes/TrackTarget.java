package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicHeadingLimit;
import frc.robot.auto.actions.MoveHatchArm;
import frc.robot.auto.actions.OpenLoopDrive;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.TurnInPlace;
import frc.robot.subsystems.HatchArm.HatchArmState;
import java.util.Arrays;

public class TrackTarget extends AutoModeBase {


  public TrackTarget() {

  }

  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new ParallelAction(Arrays.asList(
        new MotionMagicHeadingLimit(3.0),
        new MoveHatchArm(HatchArmState.PLACE)
    )));
    runAction(new ParallelAction(Arrays.asList(
        new OpenLoopDrive(-0.4, -0.4, 1.5),
        new MoveHatchArm(HatchArmState.STOW)
    )));
    runAction(new TurnInPlace(180.0, 3.0));
  }
}
