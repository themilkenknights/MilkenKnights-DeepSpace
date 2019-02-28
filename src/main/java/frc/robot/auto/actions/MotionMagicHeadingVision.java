package frc.robot.auto.actions;

import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.MkTime;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import frc.robot.subsystems.Vision;

public class MotionMagicHeadingVision implements Action {

    private MkTime timer = new MkTime();
    private double initAngle, dist = 0.0;

    public MotionMagicHeadingVision() {

    }

    @Override public boolean isFinished() {
        return timer.isDone() || HatchArm.getInstance().isHatchLimitTriggered();
    }

    @Override public void update() {
        if (Drive.getInstance().getVisionServoError(dist)) {
            HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.SPEAR_PLACE_ONLY);
        }
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
            Drive.getInstance().setDistanceAndAngle(mTarget.getDistance() + 3.0, -mTarget.getYaw() * 1.1);
            initAngle = -mTarget.getYaw();
            dist = mTarget.getDistance() + 3.0;
            timer.start(6.0);
        }
    }
}
