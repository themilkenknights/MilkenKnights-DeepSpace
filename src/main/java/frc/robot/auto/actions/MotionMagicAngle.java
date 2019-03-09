package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;

public class MotionMagicAngle implements Action {

    private double dist;
    private double angle;

    public MotionMagicAngle(double dist, double angle) {
        this.dist = dist;
        this.angle = angle;
    }

    @Override public boolean isFinished() {
        return Drive.getInstance().isMotionMagicFinished();
    }

    @Override public void update() {
    }

    @Override public void done() {
    }

    @Override public void start() {
        Drive.getInstance().setDistanceAndAngle(dist, angle);
    }
}
