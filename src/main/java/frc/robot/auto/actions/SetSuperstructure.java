package frc.robot.auto.actions;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;

public class SetSuperstructure extends RunOnceAction {
  private RobotState state;

  public SetSuperstructure(RobotState state) {
    this.state = state;
  }

  @Override
  public void runOnce() {
    Superstructure.getInstance().setRobotState(RobotState.TELEOP_DRIVE);
  }
}
