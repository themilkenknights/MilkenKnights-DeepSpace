package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class MotionMagicAngle implements Action {

    private double dist;
    private int i = 0;

    public MotionMagicAngle(double dist) {
        this.dist = dist - 10.0;
    }

    @Override public boolean isFinished() {
        if (i < 5) {
            i++;
            return false;
        }
        return Drive.getInstance().isMotionMagicFinished();
    }

    @Override public void update() {
        double ang = Drive.getInstance().getHeadingDeg() * 0.03;
        Drive.getInstance().setMotionMagicDeltaSetpoint(new DriveSignal(dist, dist, NeutralMode.Brake), new DriveSignal(ang, -ang));
    }

    @Override public void done() {

    }

    @Override public void start() {
        Drive.getInstance().zeroPigeon();
        Drive.getInstance().setMotionMagicDeltaSetpoint(new DriveSignal(dist, dist, NeutralMode.Brake), DriveSignal.BRAKE);
    }
}
