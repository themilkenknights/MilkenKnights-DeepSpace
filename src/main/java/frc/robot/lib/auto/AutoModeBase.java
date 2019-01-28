package frc.robot.lib.auto;

import frc.robot.lib.util.Logger;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in auto modes (which are routines that do actions).
 */
public abstract class AutoModeBase {

  protected double m_update_rate = 1.0 / 200.0;
  protected boolean m_active = false;

  public void run() {
    m_active = true;
    try {
      routine();
    } catch (AutoModeEndedException e) {
      Logger.logMarker("Auto mode done, ended early");
      return;
    }
    done();
    Logger.logMarker("Auto mode done");
  }

  protected abstract void routine() throws AutoModeEndedException;

  public void done() {
  }

  public void stop() {
    m_active = false;
  }

  public void runAction(Action action) throws AutoModeEndedException {
    isActiveWithThrow();
    action.start();
    while (isActiveWithThrow() && !action.isFinished()) {
      action.update();
      long waitTime = (long) (m_update_rate * 1000.0);
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
    return m_active;
  }
}
