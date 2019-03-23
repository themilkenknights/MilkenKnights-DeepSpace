package frc.robot.auto.modes;

import frc.robot.AutoChooser;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.VisionDrive;
import frc.robot.lib.util.DeserializePath;

public class SideCargoshipHatchAuto extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
  /*  runAction(new ParallelAction(Arrays.asList(
        new DrivePathAction(1, false),
        new DelayAction(getTime(1) - 0.35, new MotionMagicVisionFeed(MotionMagicVisionFeed.VisionGoal.PLACE_HATCH)))));

    runAction(new ParallelAction(Arrays.asList(
        new DrivePathAction(2, false),
        new DelayAction(getTime(2) - 0.22, new MotionMagicVisionFeed(MotionMagicVisionFeed.VisionGoal.PLACE_HATCH)))));

    runAction(new ParallelAction(Arrays.asList(
        new DrivePathAction(3, false),
        new DelayAction(getTime(3) - 0.22, new MotionMagicVisionFeed(MotionMagicVisionFeed.VisionGoal.PLACE_HATCH)))));

    runAction(new ParallelAction(Arrays.asList(
        new DrivePathAction(4, false),
        new DelayAction(getTime(4) - 0.22, new MotionMagicVisionFeed(MotionMagicVisionFeed.VisionGoal.PLACE_HATCH)))));
 */

   /* runAction(new DrivePathAction(2, true));
    runAction(new DrivePathAction(3, false));
    runAction(new DrivePathAction(4, true));
    runAction(new DrivePathAction(5, false)); */
    runAction(new DrivePathAction(1, false));
    runAction(new VisionDrive(VisionDrive.VisionGoal.PLACE_HATCH));
  }

  private double getTime(int pathNum) {
    return DeserializePath.getPathFromFile("CS-" + pathNum + (AutoChooser.mAutoPosition == AutoChooser.AutoPosition.LEFT ? "L" : "R")).getTime();
  }
}
