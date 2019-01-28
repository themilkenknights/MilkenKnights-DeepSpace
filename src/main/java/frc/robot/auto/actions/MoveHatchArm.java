package frc.robot.auto.actions;

import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchArmState;

public class MoveHatchArm implements Action {

  HatchArmState armState;

  public MoveHatchArm(HatchArmState armState) {
    this.armState = armState;
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void update() {

  }

  @Override
  public void done() {
  }

  @Override
  public void start() {
    HatchArm.getInstance().setHatchArm(armState);
  }

}
