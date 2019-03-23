package frc.robot.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class MkTimer {
  private double timerLength;
  private double startTime;
  private boolean hasBeenSet;

  public void start(double timerLength) {
    this.timerLength = timerLength;
    startTime = Timer.getFPGATimestamp();
    hasBeenSet = true;
  }

  public void reset() {
    hasBeenSet = false;
  }

  public boolean hasBeenSet() {
    return hasBeenSet;
  }

  public boolean isDone() {
    return hasBeenSet && Timer.getFPGATimestamp() - startTime >= timerLength;
  }

  public boolean isDone(double length) {
    return hasBeenSet && Timer.getFPGATimestamp() - startTime >= length;
  }
}
