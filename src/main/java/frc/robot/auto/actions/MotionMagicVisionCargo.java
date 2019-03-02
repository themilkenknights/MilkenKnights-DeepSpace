package frc.robot.auto.actions;

import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.MkTime;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import frc.robot.subsystems.HatchArm.HatchSpearState;
import frc.robot.subsystems.Vision;

public class MotionMagicVisionCargo implements Action {

    private MkTime timer = new MkTime();
    private double initAngle, dist = 0.0;
    private MkTime downTimer = new MkTime();

    public MotionMagicVisionCargo() {

    }

    @Override public boolean isFinished() {
        //System.out.println("Hatch: " + HatchArm.getInstance().isHatchLimitTriggered() + " Place " + (HatchArm.getInstance().getHatchSpearState() == HatchSpearState.PLACE) + " Timer " +  downTimer.isDone());
        return timer.isDone() || Drive.getInstance().isVisionFinished()/*|| (HatchArm.getInstance().isHatchLimitTriggered() && (HatchArm.getInstance().getHatchSpearState() == HatchSpearState.PLACE) && downTimer.isDone())*/;
    }

    @Override public void update() {
    /*    if (Drive.getInstance().getVisionServoError(dist) && (HatchArm.getInstance().getHatchSpearState() != HatchSpearState.PLACE)) {
            HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.SPEAR_PLACE_ONLY);
            downTimer.start(0.5);
        } */
       /* LimelightTarget mTarget = Vision.getInstance().getLimelightTarget();
        if (mTarget.isValidTarget()) {
            Drive.getInstance().setDistanceAndAngle(mTarget.getDistance(), initAngle);
        } */
    }

    @Override public void done() {
        Drive.getInstance().setOpenLoop(DriveSignal.BRAKE);
    }

    @Override public void start() {
        LimelightTarget mTarget = Vision.getInstance().getLimelightTarget();
        if (mTarget.isValidTarget()) {
            Drive.getInstance().setDistanceAndAngle(mTarget.getDistance() + 2.5, -mTarget.getYaw() * 1.0);
            initAngle = -mTarget.getYaw();
            dist = mTarget.getDistance() + 2.5;
            timer.start(2.85);
        }
    }
}
