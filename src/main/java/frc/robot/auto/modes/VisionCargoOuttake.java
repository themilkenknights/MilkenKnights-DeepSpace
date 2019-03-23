package frc.robot.auto.modes;

import frc.robot.Constants.CARGO_ARM;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.DelayAction;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.auto.actions.MotionMagicVisionFeed;
import frc.robot.auto.actions.OpenLoopDrive;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.RollerAction;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.Superstructure;
import java.util.Arrays;

public class VisionCargoOuttake extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new MotionMagicVisionFeed(MotionMagicVisionFeed.VisionGoal.PLACE_CARGO));
runAction(new OpenLoopDrive(-0.5, -0.5, 0.5));
runAction(new SetSuperstructure(Superstructure.RobotState.TELEOP_DRIVE));
  }


}
