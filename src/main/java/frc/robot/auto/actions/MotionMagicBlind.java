package frc.robot.auto.actions;

import frc.robot.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class MotionMagicBlind implements Action {
  private double dist;
  private int i;

  public MotionMagicBlind(double dist) {
    this.dist = dist;
  }

  @Override
  public boolean isFinished() {
    if (i < 5) {
      i++;
      return false;
    }
    return Drive.getInstance().isMotionMagicFinished();
  }

  @Override
  public void update() {
  }

  @Override
  public void done() {
  }

  @Override
  public void start() {
    Drive.getInstance().setMotionMagicPositionSetpoint(new DriveSignal(dist, dist), DriveSignal.BRAKE);
  }
}
