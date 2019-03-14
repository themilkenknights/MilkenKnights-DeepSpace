package frc.robot.auto.actions;

import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchSpearState;

public class SetHatchArmState extends RunOnceAction {

  private HatchSpearState state;

  public SetHatchArmState(HatchSpearState state) {
    this.state = state;
  }

  @Override
  public void runOnce() {
    HatchArm.getInstance().setHatchState(state);
  }
}
