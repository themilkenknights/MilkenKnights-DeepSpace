package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.CrashTrackingRunnable;
import frc.robot.lib.util.DeltaTime;
import frc.robot.subsystems.Vision;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager {

    private final List<Subsystem> mAllSubsystems;
    private final Notifier notifier_;
    private final Object taskRunningLock_ = new Object();
    private double timestamp_ = 0;
    private boolean running_;
    private int count = 0;
    private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
        @Override public void runCrashTracked() {
            synchronized (taskRunningLock_) {
                double now = fastDt.updateDt();
                if (running_) {
                    for (Subsystem subsystem : mAllSubsystems) {
                        Input.updateControlInput();
                        subsystem.readPeriodicInputs(now);
                        subsystem.onQuickLoop(now);
                        subsystem.writePeriodicOutputs(now);
                        if (count == 5) {
                            subsystem.slowUpdate(now);
                            subsystem.outputTelemetry(now);
                            subsystem.safetyCheck(now);
                        }
                    }
                    if (count == 5) {
                        count = 0;
                    }
                    count++;
                } else {
                    for (Subsystem subsystem : mAllSubsystems) {
                        subsystem.outputTelemetry(now);
                    }
                }
                Vision.getInstance().updateLimelight();
                timestamp_ = now;
            }
        }
    };
    private DeltaTime mainDt, fastDt;

    public SubsystemManager(List<Subsystem> allSubsystems) {
        mAllSubsystems = allSubsystems;

        notifier_ = new Notifier(runnable_);
        running_ = false;

        mainDt = new DeltaTime("Main", 5);
        fastDt = new DeltaTime("Fast", 5);
    }

    public void checkSystem() {
        for (Subsystem subsystem : mAllSubsystems) {
            subsystem.checkSystem();
        }
    }

    public void perioidicUpdate() {
        mainDt.updateDt();
    }

    public void startAuto() {
        double now = Timer.getFPGATimestamp();
        for (Subsystem subsystem : mAllSubsystems) {
            subsystem.autonomousInit(now);
        }

        if (!running_) {
            System.out.println("Starting Sandstorm (Auto)");
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
            System.out.println("Starting Teleop");
            running_ = true;

        }
        notifier_.startPeriodic(GENERAL.kFastLooperDt);
    }

    public void stop() {
        if (running_) {
            System.out.println("Stopping");
            running_ = false;

        }
        timestamp_ = Timer.getFPGATimestamp();
        for (Subsystem subsystem : mAllSubsystems) {
            subsystem.onStop(timestamp_);
        }
        notifier_.startPeriodic(GENERAL.kFastLooperDt);
    }

}
