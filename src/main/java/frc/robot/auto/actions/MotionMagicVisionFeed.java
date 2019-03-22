package frc.robot.auto.actions;

import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTimer;
import frc.robot.lib.util.SynchronousPIDF;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Vision;

public class MotionMagicVisionFeed implements Action {
  private MkTimer expirationTimer = new MkTimer();
  private VisionGoal mGoal;

  public MotionMagicVisionFeed(VisionGoal mGoal) {
    this.mGoal = mGoal;
  }

  @Override
  public boolean isFinished() {
    return expirationTimer.isDone() || ((mGoal == VisionGoal.INTAKE_HATCH
        || mGoal == VisionGoal.PLACE_HATCH) && (HatchArm.getInstance().isHatchTriggeredTimer()));
  }

  @Override
  public void update() {

  }

  @Override
  public void done() {
  }

  @Override
  public void start() {
    Drive.getInstance().cancelPath();
    expirationTimer.start(3.0);
    Drive.getInstance().setVisionDrive(mGoal);
  }

  public enum VisionGoal {
    PLACE_HATCH, INTAKE_HATCH, PLACE_CARGO
  }
}
