package frc.robot.auto.actions;

import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTimer;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.Vision;

public class MotionMagicVisionPigeon implements Action {

  private MkTimer timer = new MkTimer();
  private double lastDist, lastAngle, lastSkew = 0.0;
  private MkTimer downTimer = new MkTimer();
  private MkTimer driveBackTimer = new MkTimer();
  private VisionServoGoal goal;
  private boolean turnBack, isDone = false;

  public MotionMagicVisionPigeon(VisionServoGoal goal) {
    this.goal = goal;
  }

  /**
   * Exit tracking if it is taking too long or if the tracking is finished or if the arm is down for a specified time
   * and the limit switch is triggered.
   */
  @Override
  public boolean isFinished() {
    return isDone || timer.isDone()
        || Drive.getInstance().isVisionFinished(lastDist, lastAngle + lastSkew)
        || (HatchArm.getInstance().isHatchLimitTriggered()
        && ((HatchArm.getInstance().getHatchSpearState() == HatchState.PLACE)
        && downTimer.isDone()));
  }

  @Override
  public void update() {
    switch (goal) {
      case NO_ACTION:
        break;
      case PLACE_HATCH:
        if ((Drive.getInstance().getVisionServoError(lastDist) < 28.0)
            && (HatchArm.getInstance().getHatchSpearState() != HatchState.PLACE)) {
          HatchArm.getInstance().setHatchState(HatchState.PLACE);
          downTimer.start(0.75);
        }
        break;
      case INTAKE_HATCH:
        if ((Drive.getInstance().getVisionServoError(lastDist) < 60.0)
            && (HatchArm.getInstance().getHatchSpearState() != HatchState.PLACE)) {
          HatchArm.getInstance().setHatchState(HatchState.INTAKE);
          downTimer.start(1.0);
        }
        break;
      default:
        Logger.logErrorWithTrace("Unexpected Vision Servo Goal");
    }

   LimelightTarget mTarget = Vision.getInstance().getLimelightTarget();
    if (turnBack && !driveBackTimer.isDone()) {
      //Drive.getInstance().setOpenLoop(new DriveSignal(-0.3, -0.3));
      //TODO Fix
    } else if (mTarget.isValidTarget() && mTarget.getDistance() < 28.0) {
      lastAngle = -mTarget.getYaw();
      lastDist = mTarget.getDistance();
      lastSkew = mTarget.getSkew() * -2.0;
      Drive.getInstance().setDistanceAndAngle(lastDist, lastAngle + lastSkew);
    }
  }

  @Override
  public void done() {
    Superstructure.getInstance().setRobotState(RobotState.TELEOP_DRIVE);
  }

  @Override
  public void start() {
    turnBack = false;
    LimelightTarget mTarget = Vision.getInstance().getLimelightTarget();
    if (mTarget.isValidTarget()) {
      if (mTarget.getDistance() > 35 && Math.abs(mTarget.getYaw()) > 20.0) {
        turnBack = true;
        isDone = true;
        //driveBackTimer.start(1.0);
      } else {
        lastAngle = -mTarget.getYaw();
        lastDist = mTarget.getDistance();
        lastSkew = mTarget.getSkew() * -2.0;
        Drive.getInstance().setDistanceAndAngle(lastDist, lastAngle + lastSkew);
        timer.start(4.0);
      }
    } else {
      Logger.logMarker("Invalid Target");
    }
  }

  public enum VisionServoGoal {
    PLACE_HATCH,
    INTAKE_HATCH,
    PLACE_CARGO,
    NO_ACTION
  }
}
