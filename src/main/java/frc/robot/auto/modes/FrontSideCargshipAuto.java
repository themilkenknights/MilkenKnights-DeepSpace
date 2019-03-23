package frc.robot.auto.modes;

import frc.robot.AutoChooser;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.TurnInPlace;
import frc.robot.auto.actions.VisionDrive;

public class FrontSideCargshipAuto extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new DrivePathAction(1, false));
    runAction(new VisionDrive(VisionDrive.VisionGoal.PLACE_HATCH));
    runAction(new DrivePathAction(2, true));
    runAction(new TurnInPlace(180.0));
    runAction(new VisionDrive(VisionDrive.VisionGoal.INTAKE_HATCH));
    runAction(new DrivePathAction(3, true));
    runAction(new TurnInPlace(90.0));
    runAction(new VisionDrive(VisionDrive.VisionGoal.PLACE_HATCH));
  }

  private double getTime(int pathNum) {
    return AutoChooser.getPath("CS-" + pathNum + (AutoChooser.mAutoPosition == AutoChooser.AutoPosition.LEFT ? "L" : "R")).getTime();
  }
}
