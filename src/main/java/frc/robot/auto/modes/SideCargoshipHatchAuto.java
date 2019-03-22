package frc.robot.auto.modes;

import frc.robot.AutoChooser;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DelayAction;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.MotionMagicVisionFeed;
import frc.robot.auto.actions.ParallelAction;
import java.util.Arrays;

public class SideCargoshipHatchAuto extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new ParallelAction(Arrays.asList(
        new DrivePathAction(1, false),
        new DelayAction(getTime(1) - 1.0, new MotionMagicVisionFeed(MotionMagicVisionFeed.VisionGoal.PLACE_HATCH)))));
  }

  private double getTime(int pathNum) {
    return AutoChooser.autoPaths.get("CS-" + pathNum + (AutoChooser.mAutoPosition == AutoChooser.AutoPosition.LEFT ? "L" : "R")).getTime();
  }
}
