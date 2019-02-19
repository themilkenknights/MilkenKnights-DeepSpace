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
	private final Notifier _pixyUpdate;
	private final Notifier _limelightUpdate;
	private double timestamp_ = 0;
	private boolean running_;
	private DeltaTime mainDt, slowDt, fastDt, llDt, pixyDt, telemetryDt;

	public SubsystemManager(List<Subsystem> allSubsystems) {
		mAllSubsystems = allSubsystems;

		notifier_ = new Notifier(runnable_);
		_pixyUpdate = new Notifier(pixyRunnable_);
		_limelightUpdate = new Notifier(limelightRunnable_);
		running_ = false;

		mainDt = new DeltaTime("Main", 5);
		fastDt = new DeltaTime("Fast", 5);
		llDt = new DeltaTime("Limelight", 5);
		pixyDt = new DeltaTime("Pixy", 5);
	}


	public void mainLoop() {
		double now = mainDt.start();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.onMainLoop(now);
		}
		mainDt.updateDt();
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
			System.out.println("Starting Auto loops");
			running_ = true;

		}
		notifier_.startPeriodic(GENERAL.kFastLooperDt);
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
		_limelightUpdate.startPeriodic(GENERAL.kLimelightLoopPeriod);
	}

	public synchronized void startTeleop() {
		double now = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.teleopInit(now);
		}
		if (!running_) {
			System.out.println("Starting Teleop loops");
			running_ = true;

		}
		notifier_.startPeriodic(GENERAL.kFastLooperDt);
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
		_limelightUpdate.startPeriodic(GENERAL.kLimelightLoopPeriod);
	}

	public synchronized void stop() {
		if (running_) {
			System.out.println("Stopping loops");
			running_ = false;

		}
		notifier_.stop();
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
		_limelightUpdate.startPeriodic(GENERAL.kLimelightLoopPeriod);

		timestamp_ = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.onStop(timestamp_);
		}
	}

	private final CrashTrackingRunnable pixyRunnable_ = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			pixyDt.start();
			//MkPixy.pixyUpdate();
			pixyDt.updateDt();
		}
	};

	private final CrashTrackingRunnable limelightRunnable_ = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			llDt.start();
			Vision.mLimeLight.threadUpdate();
			llDt.updateDt();
		}
	};

	private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
		private int count = 0;

		@Override
		public void runCrashTracked() {
			if (running_) {
				double now = fastDt.start();
				for (Subsystem subsystem : mAllSubsystems) {
					Input.updateControlInput();
					subsystem.readPeriodicInputs(now);
					subsystem.onQuickLoop(now);
					subsystem.writePeriodicOutputs(now);
					subsystem.safetyCheck(now);
					if (count == 5) {
						subsystem.slowUpdate(now);
						subsystem.outputTelemetry(now);
						count = 0;
					}
					count++;
				}
				timestamp_ = now;
				fastDt.updateDt();
			}
		}
	};

}
