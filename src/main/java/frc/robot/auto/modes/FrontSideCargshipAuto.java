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

    runAction(new VisionDrive(VisionDrive.VisionGoal.PLACE_HATCH));
    runAction(new ParallelAction(Arrays.asList(
        new DrivePathAction(2, true),
        new DelayAction(1.0, new SetHatchArmState(HatchArm.HatchState.INTAKE)),
        new DelayAction(getTime(2) - 0.05, new DrivePathAction(3, false)),
        new DelayAction(getTime(2) + getTime(3) - 1.4, new VisionDrive(VisionDrive.VisionGoal.INTAKE_HATCH))
    )));

    runAction(new ParallelAction(Arrays.asList(
        new DrivePathAction(4, true),
        new DelayAction(getTime(1) - 0.1, new VisionDrive(VisionDrive.VisionGoal.PLACE_HATCH)))));

    runAction(new OpenLoopDrive(-0.5, -0.5, 0.5));
    runAction(new SetSuperstructure(Superstructure.RobotState.TELEOP_DRIVE));
  }

  private double getTime(int pathNum) {
    return AutoChooser.getPath("CS-" + pathNum + (AutoChooser.mAutoPosition == AutoChooser.AutoPosition.LEFT ? "L" : "R")).getTime();
  }
}
