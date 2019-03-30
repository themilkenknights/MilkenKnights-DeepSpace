package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.OpenLoopDrive;
import frc.robot.auto.actions.SetHatchArmState;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.auto.actions.VisionDrive;
import frc.robot.auto.actions.WaitAction;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Superstructure;

public class HatchIntakeVisionPigeon extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new VisionDrive(VisionDrive.VisionGoal.INTAKE_HATCH));
    runAction(new WaitAction(0.25));
    runAction(new OpenLoopDrive(-0.65, -0.65, 0.5));
    runAction(new SetHatchArmState(HatchArm.HatchState.STOW));
    runAction(new SetSuperstructure(Superstructure.RobotState.TELEOP_DRIVE));
  }
}
