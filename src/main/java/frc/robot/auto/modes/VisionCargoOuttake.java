package frc.robot.auto.modes;

import frc.robot.Constants;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.OpenLoopDrive;
import frc.robot.auto.actions.RollerAction;
import frc.robot.auto.actions.SetSuperstructure;
import frc.robot.auto.actions.VisionDrive;
import frc.robot.subsystems.Superstructure;

public class VisionCargoOuttake extends AutoModeBase {
  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new VisionDrive(VisionDrive.VisionGoal.PLACE_CARGO));
    runAction(new RollerAction(Constants.CARGO_ARM.kCargoShipIntakeRollerOut, 0.35));
    runAction(new OpenLoopDrive(-0.5, -0.5, 0.4));
    runAction(new SetSuperstructure(Superstructure.RobotState.TELEOP_DRIVE));
  }
}
