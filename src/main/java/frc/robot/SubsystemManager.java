package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.geometry.RobotState;
import frc.robot.lib.util.CrashTrackingRunnable;
import frc.robot.lib.util.DeltaTime;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.Subsystem;
import frc.robot.subsystems.Vision;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager {
  private final List<Subsystem> mAllSubsystems;
  private final Notifier notifier_;
  private final Object taskRunningLock_ = new Object();
  private double timestamp_;
  private boolean running_;
  private int count;
  /**
   * Control Input, periodic inputs/outputs, and quick updates should be run every cycle (20ms) Slow updates, telemetry, and safety checks should be
   * run approx every 100ms. One subsystem should update each cycle to extreme minimize delays. Vision updates should occur every 20ms regardless of
   * enabled state. While disbaled, run telemetry at 50hz (every 20ms) as there is no need to slow it down
   */
  private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
    @Override
    public void runCrashTracked() {
      synchronized (taskRunningLock_) {
        double now = fastDt.updateDt();
        Vision.getInstance().updateLimelight();
        if (running_) {
          int i = 0;
          for (Subsystem subsystem : mAllSubsystems) {
            Input.updateControlInput();
            subsystem.readPeriodicInputs(now);
            subsystem.onQuickLoop(now);
            subsystem.writePeriodicOutputs(now);
            if (count == i) {
              subsystem.outputTelemetry(now);
              subsystem.safetyCheck(now);
            }
            if (count == 3) {
              RobotState.getInstance().outputToSmartDashboard();
            }
            i++;
          }
        } else {
          int i = 0;
          for (Subsystem subsystem : mAllSubsystems) {
            if (count == i) {
              subsystem.readPeriodicInputs(now);
              subsystem.outputTelemetry(now);
            }
            i++;
          }
        }
        if (count == mAllSubsystems.size()) {
          count = 0;
        }
        count++;
        timestamp_ = now;
      }
    }
  };
  private DeltaTime mainDt, fastDt;

  public SubsystemManager(List<Subsystem> allSubsystems) {
    mAllSubsystems = allSubsystems;
    notifier_ = new Notifier(runnable_);
    running_ = false;
    mainDt = new DeltaTime("Iterative Dt", 1, false);
    fastDt = new DeltaTime("Looper Dt", 1, false);
  }

  public void checkSystem() {
    boolean check = true;
    for (Subsystem subsystem : mAllSubsystems) {
      check &= subsystem.checkSystem();
    }
    if (check) {
      Logger.logMarker("Test Success");
    } else {
      Logger.logError("TEST FAILED!!!!!");
    }
  }

  public void init() {
    for (Subsystem subsystem : mAllSubsystems) {
      subsystem.onRestart(Timer.getFPGATimestamp());
    }
  }

  public void periodicUpdate() {
    mainDt.updateDt();
  }

  public void startAuto() {
    double now = Timer.getFPGATimestamp();
    for (Subsystem subsystem : mAllSubsystems) {
      subsystem.autonomousInit(now);
    }
    if (!running_) {
      Logger.logMarker("Starting Sandstorm (Auto)");
      running_ = true;
    }
    notifier_.startPeriodic(GENERAL.kFastLooperDt);
  }

  public void startTeleop() {
    double now = Timer.getFPGATimestamp();
    for (Subsystem subsystem : mAllSubsystems) {
      subsystem.teleopInit(now);
    }
    if (!running_) {
      Logger.logMarker("Starting Teleop");
      running_ = true;
    }
    notifier_.startPeriodic(GENERAL.kFastLooperDt);
  }

  /**
   * Never disable the main loop, simply set the running boolean to false to stop outputs
   */
  public void stop() {
    if (running_) {
      Logger.logMarker("Stopping Loops");
      running_ = false;
    }
    timestamp_ = Timer.getFPGATimestamp();
    for (Subsystem subsystem : mAllSubsystems) {
      subsystem.onStop(timestamp_);
    }
    notifier_.startPeriodic(GENERAL.kFastLooperDt);
  }
}
