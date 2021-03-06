package frc.robot.auto;

import frc.robot.auto.actions.Action;
import frc.robot.lib.util.DeltaTime;
import frc.robot.lib.util.Logger;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are routines that do actions).
 */
public abstract class AutoModeBase {
  protected double mUpdateRate = 1.0 / 200.0;
  protected boolean mActive;
  private DeltaTime baseTimer = new DeltaTime("Auto Base Dt", 1, true);

  public void run() {
    mActive = true;
    try {
      routine();
    } catch (AutoModeEndedException e) {
      Logger.logMarker("AUTO MODE DONE!!!!");
      return;
    }
    done();
  }

  protected abstract void routine() throws AutoModeEndedException;

  public void done() {
    Logger.logMarker("Auto mode done");
  }

  public void stop() {
    mActive = false;
  }

  public void runAction(Action action) throws AutoModeEndedException {
    isActiveWithThrow();
    action.start();
    while (isActiveWithThrow() && !action.isFinished()) {
      baseTimer.updateDt();
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
