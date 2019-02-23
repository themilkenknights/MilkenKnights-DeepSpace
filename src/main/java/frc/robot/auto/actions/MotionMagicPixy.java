package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.VISION;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.MkTime;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;

public class MotionMagicPixy implements Action {

    private MkTime expirationTimer;

    public MotionMagicPixy() {
        expirationTimer = new MkTime();
    }

    @Override public boolean isFinished() {
        return expirationTimer.isDone() || Vision.getInstance().getPixyTarget().isCargoIntaked();
    }

    @Override public void update() {
        double mDist = VISION.kPixyAreaToDistVisionMap.getInterpolated(new InterpolatingDouble((Vision.getInstance().getPixyTarget().getArea()))).value;
        if (mDist > 5.0 && mDist < 60) {
            double mSteer = DRIVE.kPixyKp * Vision.getInstance().getPixyTarget().getYaw();
            Drive.getInstance().setMotionMagicDeltaSetpoint(new DriveSignal(mDist, mDist, NeutralMode.Coast), new DriveSignal(mSteer, -mSteer));
        } else {
            Drive.getInstance().setOpenLoop(new DriveSignal(-0.3, -0.3));
        }
    }

    @Override public void done() {

    }

    @Override public void start() {
        expirationTimer.start(5.0);
    }
}
