package frc.robot.auto.actions;

import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchState;

public class SetHatchArmState extends RunOnceAction {

  private HatchState state;

  public SetHatchArmState(HatchState state) {
    this.state = state;
  }

  @Override
  public void runOnce() {
    HatchArm.getInstance().setHatchState(state);
  }
}
