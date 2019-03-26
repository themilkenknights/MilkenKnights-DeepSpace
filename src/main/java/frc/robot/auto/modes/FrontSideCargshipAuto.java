package frc.robot.auto.modes;

import frc.robot.AutoChooser;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DelayAction;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.OpenLoopDrive;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.SetHatchArmState;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.auto.actions.VisionDrive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Superstructure;
import java.util.Arrays;

public class FrontSideCargshipAuto extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {

   runAction(new OpenLoopDrive(0.35, 0.35, 0.5));
    runAction(new VisionDrive(VisionDrive.VisionGoal.PLACE_HATCH, 0.5));

    runAction(new ParallelAction(Arrays.asList(
        new DrivePathAction(1, true),
        new DelayAction(getTime(1) + 0.5, new SetHatchArmState(HatchArm.HatchState.INTAKE)),
        new DelayAction(getTime(1) - 0.1, new DrivePathAction(2, false)),
        new DelayAction(getTime(1) + getTime(2) - 0.39, new VisionDrive(VisionDrive.VisionGoal.INTAKE_HATCH, 0.9))
    )));

    runAction(new ParallelAction(Arrays.asList(
        new DrivePathAction(3, true),
        new DelayAction(getTime(3) - 0.175, new VisionDrive(VisionDrive.VisionGoal.PLACE_HATCH, 0.5)))));

    runAction(new OpenLoopDrive(-0.5, -0.5, 0.5));
    runAction(new SetSuperstructure(Superstructure.RobotState.TELEOP_DRIVE));
  }

  private double getTime(int pathNum) {
    return AutoChooser.getPath("CS-" + pathNum + (AutoChooser.mAutoPosition == AutoChooser.AutoPosition.LEFT ? "L" : "R")).getTime();
  }
}
