package frc.robot.lib.util;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DeltaTime {
  private double lastTime;
  private int loopsTillUpdate, count;
  private NetworkTableEntry mEntry;
  private boolean smartDash;
  private String name;

  public DeltaTime(String name, int loopsTillUpdate, boolean smartDash) {
    this.loopsTillUpdate = loopsTillUpdate;
    this.name = name;
    count = 0;
    this.smartDash = smartDash;
    if (!smartDash) {
      mEntry = Shuffleboard.getTab("General").add(name, 0.0).getEntry();
    }
  }

  public double updateDt() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    lastTime = now;
    if (count == loopsTillUpdate) {
      if (smartDash) {
        SmartDashboard.putNumber(name, dt * 1e3);
      } else {
        mEntry.setDouble(dt * 1e3);
      }
      count = 0;
    } else {
      count++;
    }
    return now;
  }
}
