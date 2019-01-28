package frc.robot.lib.structure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager {

  private final List<Subsystem> mAllSubsystems;
  private double lastTime = 0;

  public SubsystemManager(List<Subsystem> allSubsystems) {
    mAllSubsystems = allSubsystems;
  }

  public void outputToSmartDashboard() {
    double timestamp_ = Timer.getFPGATimestamp();
    if (timestamp_ - lastTime >= 0.05) {
      mAllSubsystems.forEach((s) -> s.outputTelemetry());
      SmartDashboard.putNumber("Main loop Dt", (timestamp_ - lastTime) * 1e3);
    }
    lastTime = timestamp_;
  }

  public void onLoop() {
    double timestamp_ = Timer.getFPGATimestamp();
    for (Subsystem subsystem : mAllSubsystems) {
      subsystem.readPeriodicInputs(timestamp_);
      subsystem.onLoop(timestamp_);
      subsystem.writePeriodicOutputs(timestamp_);
    }
  }

  public void start() {
    double timestamp_ = Timer.getFPGATimestamp();
    for (Subsystem subsystem : mAllSubsystems) {
      subsystem.onStart(timestamp_);
    }
  }

  public void stop() {
    double timestamp_ = Timer.getFPGATimestamp();
    for (Subsystem subsystem : mAllSubsystems) {
      subsystem.onStop(timestamp_);
    }
  }
}
