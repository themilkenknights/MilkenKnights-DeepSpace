package frc.robot.auto.actions;

import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.CargoArmState;

public class CargoArmSetpoint extends RunOnceAction {
  private CargoArmState state;

  public CargoArmSetpoint(CargoArmState state) {
    this.state = state;
  }

  @Override
  public void runOnce() {
    CargoArm.getInstance().setArmState(state);
  }
}
