package frc.robot.lib.structure;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.util.CrashTrackingRunnable;
import frc.robot.lib.util.Logger;
import frc.robot.lib.vision.MkPixy;
import frc.robot.paths.RobotState;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager {

	private final List<Subsystem> mAllSubsystems;
	private final Notifier notifier_;
	private final Notifier slow_notifier_;
	private final Notifier telemetry_notifier_;
	private final Notifier _pixyUpdate;
	private final Object taskRunningLock_ = new Object();
	private final double kPeriod = GENERAL.kLooperDt;
	private final double kSlowPeriod = GENERAL.kSlowLooperDt;
	private final double kTelemetryPeriod = GENERAL.kTelemetryDt;
	private boolean running_;
	private final CrashTrackingRunnable slowRunnable_ = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			synchronized (taskRunningLock_) {
				if (running_) {
					double now = Timer.getFPGATimestamp();
					for (Subsystem subsystem : mAllSubsystems) {
						subsystem.safetyCheck(now);
					}
				}
			}
		}
	};
	private double main_timestamp = 0;
	private double timestamp_ = 0;
	private double dt_ = 0;
	private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			synchronized (taskRunningLock_) {
				if (running_) {
					double now = Timer.getFPGATimestamp();
					for (Subsystem subsystem : mAllSubsystems) {
						subsystem.readPeriodicInputs(now);
						subsystem.onQuickLoop(now);
						subsystem.writePeriodicOutputs(now);
					}
					dt_ = now - timestamp_;
					timestamp_ = now;
				}
			}
		}
	};
	private double main_loop_dt_ = 0;
	private final CrashTrackingRunnable telemetryRunnable_ = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			synchronized (taskRunningLock_) {
				double now = Timer.getFPGATimestamp();
				for (Subsystem subsystem : mAllSubsystems) {
					subsystem.outputTelemetry(now);
					RobotState.getInstance().outputToSmartDashboard();
					SmartDashboard.putNumber("Main loop Dt", main_loop_dt_ * 1e3);
					SmartDashboard.putNumber("looper_dt", dt_);
				}
			}
		}
	};

	public SubsystemManager(List<Subsystem> allSubsystems) {
		mAllSubsystems = allSubsystems;

		notifier_ = new Notifier(runnable_);
		_pixyUpdate = new Notifier(new PeriodicRunnable());
		slow_notifier_ = new Notifier(slowRunnable_);
		telemetry_notifier_ = new Notifier(telemetryRunnable_);
		running_ = false;
	}

	public void mainLoop() {
		double now = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.onMainLoop(timestamp_);
		}
		main_loop_dt_ = now - main_timestamp;
		main_timestamp = now;
	}

	public void checkSystem() {
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.checkSystem();
		}
	}

	public synchronized void startAuto() {
		double now = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.autonomousInit(now);
		}

		if (!running_) {
			synchronized (taskRunningLock_) {
				System.out.println("Starting Auto loops");
				running_ = true;
			}
		}
		notifier_.startPeriodic(kPeriod);
		slow_notifier_.startPeriodic(kSlowPeriod);
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
		telemetry_notifier_.startPeriodic(kTelemetryPeriod);
	}

	public synchronized void startTeleop() {
		double now = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.teleopInit(now);
		}
		if (!running_) {
			synchronized (taskRunningLock_) {
				System.out.println("Starting Teleop loops");
				running_ = true;
			}
		}
		notifier_.startPeriodic(kPeriod);
		slow_notifier_.startPeriodic(kSlowPeriod);
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
		telemetry_notifier_.startPeriodic(kTelemetryPeriod);
	}

	public synchronized void stop() {
		if (running_) {
			synchronized (taskRunningLock_) {
				System.out.println("Stopping loops");
				running_ = false;
			}
		}
		notifier_.stop();
		slow_notifier_.stop();
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
		telemetry_notifier_.startPeriodic(kTelemetryPeriod);

		timestamp_ = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.onStop(timestamp_);
		}
	}

	class PeriodicRunnable implements java.lang.Runnable {

		public void run() {
			try {
				MkPixy.pixyUpdate();
			} catch (Throwable t) {
				Logger.logThrowableCrash(t);
				throw t;
			}
		}
	}

}
