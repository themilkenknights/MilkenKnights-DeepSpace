package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto.actions.Action;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are routines that do
 * actions).
 */
public abstract class AutoModeBase {
    protected double mUpdateRate = 1.0 / 50.0;
    protected boolean mActive = false;

    public void run() {
        mActive = true;
        try {
            routine();
        } catch (AutoModeEndedException e) {
            DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
            return;
        }
        done();
    }

    protected abstract void routine() throws AutoModeEndedException;

    public void done() {
        System.out.println("Auto mode done");
    }

    public void stop() {
        mActive = false;
    }

    public void runAction(Action action) throws AutoModeEndedException {
        isActiveWithThrow();
        action.start();
        while (isActiveWithThrow() && !action.isFinished()) {
            action.update();
            long waitTime = (long) (mUpdateRate * 1000.0);
            try {
                Thread.sleep(waitTime);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        action.done();
    }

    public boolean isActiveWithThrow() throws AutoModeEndedException {
        if (!isActive()) {
            throw new AutoModeEndedException();
        }
        return isActive();
    }

    public boolean isActive() {
        return mActive;
    }
}
