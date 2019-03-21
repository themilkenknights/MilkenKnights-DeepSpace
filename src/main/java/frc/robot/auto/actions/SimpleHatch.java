package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.MkTimer;
import frc.robot.lib.util.SynchronousPIDF;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Vision;

public class SimpleHatch implements Action {
  private SynchronousPIDF mVisionAssist;
  private MkTimer driveTimer = new MkTimer();
  private double lastTime = 0.0;

  public SimpleHatch() {
    mVisionAssist = new SynchronousPIDF(0.0164, 0.0, 255.0);
  }

  @Override
  public boolean isFinished() {
    return (HatchArm.getInstance().isHatchLimitTriggered() && HatchArm.getInstance().getHatchSpearState() == HatchArm.HatchState.PLACE) || driveTimer.isDone();
  }

  @Override
  public void update() {
    double visionTurn = 0.0;
    LimelightTarget target = Vision.getInstance().getLimelightTarget();
    if (target.isValidTarget()) {
      if (target.getDistance() < 31.0) {
        HatchArm.getInstance().setHatchState(HatchArm.HatchState.PLACE);
      }
      if (HatchArm.getInstance().getHatchSpearState() != HatchArm.HatchState.PLACE) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastTime;
        visionTurn = mVisionAssist.calculate(target.getYaw(), dt);
        lastTime = now;
        System.out.println(dt);
      }
      Drive.getInstance().setOpenLoop(new DriveSignal(0.25 - visionTurn, 0.25 + visionTurn));
    } else {
      Drive.getInstance().setOpenLoop(DriveSignal.BRAKE);
    }
  }

  @Override
  public void done() {
    Drive.getInstance().setOpenLoop(DriveSignal.BRAKE);
  }

  @Override
  public void start() {
    driveTimer.start(3.0);
  }
}
