package frc.robot.lib.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class DeltaTime {

  private double lastTime = 0.0;
  private int loopsTillUpdate, count;
  private NetworkTableEntry mEntry;

  public DeltaTime(String name, int loopsTillUpdate) {
    this.loopsTillUpdate = loopsTillUpdate;
    mEntry = Shuffleboard.getTab("Loops").add(name, 0.0).getEntry();
    count = 0;
  }

  public double updateDt() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;
    // if (count == loopsTillUpdate) {
    mEntry.setDouble(dt * 1e3);
    //   count = 0;
    //   }
    // count++;
    return now;
  }
}
