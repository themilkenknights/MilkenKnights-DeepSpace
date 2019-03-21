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
  private SynchronousPIDF mVisionAssist = new SynchronousPIDF(0.0151, 0.0, 285.0);
  private VisionGoal mGoal;
  private double lastDist = 0.0;
  private boolean mLowered = false;

  public MotionMagicVisionFeed(VisionGoal mGoal) {
    this.mGoal = mGoal;
  }

  @Override
  public boolean isFinished() {
    return expirationTimer.isDone() || Drive.getInstance().isMotionMagicFinished() || ((mGoal == VisionGoal.INTAKE_HATCH
        || mGoal == VisionGoal.PLACE_HATCH) && (HatchArm.getInstance().isHatchTriggeredTimer()));
  }

  @Override
  public void update() {
    switch (mGoal) {
      case PLACE_HATCH:
        LimelightTarget target = Vision.getInstance().getLimelightTarget();
        if (target.isValidTarget() && target.getDistance() < 36.0 && !mLowered) {
          HatchArm.getInstance().setHatchState(HatchArm.HatchState.PLACE);
          mLowered = true;
        }
        break;
      case PLACE_CARGO:
        break;
      case INTAKE_HATCH:
        break;
      default:
        Logger.logErrorWithTrace("Unknown Vision Goal");
        break;
    }
  }

  @Override
  public void done() {
  }

  @Override
  public void start() {
    expirationTimer.start(3.0);
    Drive.getInstance().setVisionDrive();
  }

  public enum VisionGoal {
    PLACE_HATCH, INTAKE_HATCH, PLACE_CARGO
  }
}
