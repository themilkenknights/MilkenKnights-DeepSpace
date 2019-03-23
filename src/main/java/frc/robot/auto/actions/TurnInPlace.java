package frc.robot.auto.actions;

import frc.robot.lib.geometry.Rotation2d;
import frc.robot.subsystems.Drive;

public class TurnInPlace implements Action {
  private double angle;

  public TurnInPlace(double angle) {
    this.angle = angle;
  }

  @Override
  public boolean isFinished() {
    return Drive.getInstance().isDriveStateFinished();
  }

  @Override
  public void update() {
  }

  @Override
  public void done() {
  }

  @Override
  public void start() {
    Drive.getInstance().setWantTurnToHeading(Rotation2d.fromDegrees(angle));
  }
}
