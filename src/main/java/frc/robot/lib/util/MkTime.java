package frc.robot.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class MkTime {

  private double timerLength;
  private double startTime;

  public void start(double timerLength) {
    this.timerLength = timerLength;
    startTime = Timer.getFPGATimestamp();
  }


  public boolean isDone() {
    return Timer.getFPGATimestamp() - startTime >= timerLength;
  }

}
