package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CargoArmSetpoint;
import frc.robot.auto.actions.DrivePathVision;
import frc.robot.auto.actions.MotionMagicVisionFeed;
import frc.robot.auto.actions.OpenLoopDrive;
import frc.robot.auto.actions.SetHatchArmState;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Superstructure;

public class HatchOuttakeVisionPigeon extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
   // runAction(new DrivePathVision());
   runAction(new MotionMagicVisionFeed(MotionMagicVisionFeed.VisionGoal.PLACE_HATCH));
    runAction(new OpenLoopDrive(-0.5, -0.5, 0.5));
    runAction(new SetHatchArmState(HatchArm.HatchState.STOW));
    runAction(new SetSuperstructure(Superstructure.RobotState.TELEOP_DRIVE));
  }
}
