package frc.robot.auto.actions;

import frc.robot.lib.util.MkTime;
import frc.robot.subsystems.CargoArm;

public class RollerAction implements Action {

  private MkTime timer;
  private double time;
  private double output;

  public RollerAction(double output, double time) {
    timer = new MkTime();
    this.time = time;
    this.output = output;
  }

  @Override
  public boolean isFinished() {
    return timer.isDone();
  }

  @Override
  public void update() {
    CargoArm.getInstance().setIntakeRollers(output);
  }

  @Override
  public void done() {}

  @Override
  public void start() {
    timer.start(time);
  }
}
