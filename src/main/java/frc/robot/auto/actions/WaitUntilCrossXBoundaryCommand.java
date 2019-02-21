package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.paths.RobotState;

public class WaitUntilCrossXBoundaryCommand implements Action {

    private double mXBoundary = 0;

    public WaitUntilCrossXBoundaryCommand(double x) {
        mXBoundary = x;
    }

    @Override public boolean isFinished() {
        return RobotState.getInstance().getFieldToVehicle(Timer.getFPGATimestamp()).getTranslation().x() > mXBoundary;
    }

    @Override public void update() {
    }

    @Override public void done() {
    }

    @Override public void start() {
    }
}
