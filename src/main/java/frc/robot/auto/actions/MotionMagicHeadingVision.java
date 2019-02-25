package frc.robot.auto.actions;

import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class MotionMagicHeadingVision implements Action {

    public MotionMagicHeadingVision() {

    }

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void update() {
        LimelightTarget mTarget = Vision.getInstance().getLimelightTarget();
        
    }

    @Override public void done() {

    }

    @Override public void start() {
        Drive.getInstance().configHatchVision();
    }
}
