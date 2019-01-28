package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class MotionMagicHeadingLimit implements Action {

  private double desiredTime,startTime = 0;

  public MotionMagicHeadingLimit(double desiredTime) {
    this.desiredTime = desiredTime;
  }

  @Override
  public boolean isFinished() {
    return Drive.getInstance().visionTrackingDone() || (Timer.getFPGATimestamp() - startTime) > desiredTime;
  }

  @Override
  public void update() {

  }

  @Override
  public void done() {
    Drive.getInstance().setOpenLoop(DriveSignal.BRAKE);
  }

  @Override
  public void start() {
    System.out.println("Vision Tracking");
    startTime = Timer.getFPGATimestamp();
    Drive.getInstance().startVisionTracking();
  }
}

