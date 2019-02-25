package frc.robot.auto.actions;

import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Vision;

public class MotionMagicHeadingVision implements Action {

    public MotionMagicHeadingVision() {

    }

    @Override public boolean isFinished() {
        return Drive.getInstance().isMotionMagicFinished() || HatchArm.getInstance().isHatchLimitTriggered();
    }

    @Override public void update() {
        LimelightTarget mTarget = Vision.getInstance().getLimelightTarget();
        if (mTarget.isValidTarget()) {
            Drive.getInstance().setVisionSetpoint(mTarget.getDistance(), mTarget.getYaw());
        }
    }

    @Override public void done() {

    }

    @Override public void start() {

    }
}
