package frc.robot.auto.actions;

import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTimer;
import frc.robot.lib.util.SynchronousPIDF;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Vision;

public class MotionMagicVisionFeed implements Action {
  private MkTimer expirationTimer;
  private SynchronousPIDF mVisionAssist;
  private VisionGoal mGoal;
  private double lastDist = 0.0;

  public MotionMagicVisionFeed(VisionGoal mGoal) {
    expirationTimer = new MkTimer();
    mVisionAssist = new SynchronousPIDF(0.0151, 0.0, 285.0);
    this.mGoal = mGoal;
  }

  @Override
  public boolean isFinished() {
    //TODO Fix
    return expirationTimer.isDone() || Drive.getInstance().isMotionMagicFinished() || ((mGoal == VisionGoal.INTAKE_HATCH || mGoal == VisionGoal.PLACE_HATCH) && (HatchArm.getInstance().isHatchLimitTriggered()));
  }

  @Override
  public void update() {
    LimelightTarget target = Vision.getInstance().getLimelightTarget();
    if (target.isValidTarget()) {
      double visionTurn = 0.0;
      switch (mGoal) {
        case PLACE_HATCH:
          if (target.getDistance() < 36.0) {
            HatchArm.getInstance().setHatchState(HatchArm.HatchState.PLACE);
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

      if (HatchArm.getInstance().getHatchSpearState() != HatchArm.HatchState.PLACE && ((lastDist - target.getDistance()) > -0.5)) {
        visionTurn = mVisionAssist.calculate(target.getYaw());
      }

      double dist = target.getDistance();
      Drive.getInstance().setMotionMagicPositionSetpoint(dist, new DriveSignal(-visionTurn, visionTurn));
      lastDist = dist;
    }
  }

  @Override
  public void done() {
  }

  @Override
  public void start() {
    expirationTimer.start(3.0);
  }

  public enum VisionGoal {
    PLACE_HATCH, INTAKE_HATCH, PLACE_CARGO
  }
}
