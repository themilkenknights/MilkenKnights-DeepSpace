package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.CrashTrackingRunnable;
import frc.robot.paths.RobotState;
import frc.robot.subsystems.Vision;
import java.util.List;
import java.util.Map;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager {

	private final List<Subsystem> mAllSubsystems;
	private final Notifier notifier_;
	private final Notifier slow_notifier_;
	private final Notifier telemetry_notifier_;
	private final Notifier _pixyUpdate;
	private final Notifier _limelightUpdate;
	private final double kPeriod = GENERAL.kLooperDt;
	private final double kSlowPeriod = GENERAL.kSlowLooperDt;
	private final double kTelemetryPeriod = GENERAL.kTelemetryDt;
	private boolean running_;
	private ShuffleboardTab mLoopsTab;
	private NetworkTableEntry mMain, mFast, mSlow, mTelemetry, mLL;

	private final CrashTrackingRunnable pixyRunnable_ = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			//MkPixy.pixyUpdate();
		}
	};
	private double lltimestamp_ = 0;
	private double lldt_ = 0;
	private final CrashTrackingRunnable limelightRunnable_ = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			double now = Timer.getFPGATimestamp();
			Vision.mLimeLight.threadUpdate();
			lldt_ = now - lltimestamp_;
			lltimestamp_ = now;
		}
	};

	private double slowtimestamp_ = 0;
	private double slowdt_ = 0;
	private final CrashTrackingRunnable slowRunnable_ = new CrashTrackingRunnable() {
		private boolean change = true;

		@Override
		public void runCrashTracked() {
			if (running_) {
				double now = Timer.getFPGATimestamp();
				for (Subsystem subsystem : mAllSubsystems) {
					if (change) {
						//subsystem.safetyCheck(now);
					}
					change = !change;
					subsystem.slowUpdate(now);
				}
				slowdt_ = now - slowtimestamp_;
				slowtimestamp_ = now;
			}
		}
	};

	private double timestamp_ = 0;
	private double dt_ = 0;
	private final CrashTrackingRunnable runnable_ = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
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
	};
	private double telemetrytimestamp_ = 0;
	private double telemetrydt_ = 0;
	private final CrashTrackingRunnable telemetryRunnable_ = new CrashTrackingRunnable() {
		@Override
		public void runCrashTracked() {
			double now = Timer.getFPGATimestamp();
			for (Subsystem subsystem : mAllSubsystems) {
				subsystem.outputTelemetry(now);
			}
			RobotState.getInstance().outputToSmartDashboard();
			mMain.setDouble(main_loop_dt_ * 1e3);
			mFast.setDouble(dt_ * 1e3);
			mSlow.setDouble(slowdt_ * 1e3);
			mTelemetry.setDouble(telemetrydt_ * 1e3);
			mLL.setDouble(lldt_ * 1e3);
			telemetrydt_ = now - telemetrytimestamp_;
			telemetrytimestamp_ = now;
		}
	};

	public SubsystemManager(List<Subsystem> allSubsystems) {
		mAllSubsystems = allSubsystems;

		notifier_ = new Notifier(runnable_);
		_pixyUpdate = new Notifier(pixyRunnable_);
		_limelightUpdate = new Notifier(limelightRunnable_);
		slow_notifier_ = new Notifier(slowRunnable_);
		telemetry_notifier_ = new Notifier(telemetryRunnable_);
		running_ = false;

		mLoopsTab = Shuffleboard.getTab("Loops");
		mMain = mLoopsTab.add("Main", 0.0)/*.withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible Time", 5))*/.getEntry();
		mFast = mLoopsTab.add("Fast", 0.0)/*.withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible Time", 5))*/.getEntry();
		mSlow = mLoopsTab.add("Slow", 0.0)/*.withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible Time", 5))*/.getEntry();
		mTelemetry = mLoopsTab.add("Telemetry", 0.0)/*.withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible Time", 5))*/.getEntry();
		mLL = mLoopsTab.add("Limelight", 0.0)/*.withWidget(BuiltInWidgets.kGraph).withProperties(Map.of("Visible Time", 5))*/.getEntry();
	}

	private double main_loop_dt_ = 0;
	private double main_timestamp = 0;
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
			System.out.println("Starting Auto loops");
			running_ = true;

		}
		notifier_.startPeriodic(kPeriod);
		slow_notifier_.startPeriodic(kSlowPeriod);
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
		_limelightUpdate.startPeriodic(GENERAL.kLimelightLoopPeriod);
		telemetry_notifier_.startPeriodic(kTelemetryPeriod);
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
		notifier_.startPeriodic(kPeriod);
		slow_notifier_.startPeriodic(kSlowPeriod);
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
		_limelightUpdate.startPeriodic(GENERAL.kLimelightLoopPeriod);
		telemetry_notifier_.startPeriodic(kTelemetryPeriod);
	}

	public synchronized void stop() {
		if (running_) {
			System.out.println("Stopping loops");
			running_ = false;

		}
		notifier_.stop();
		slow_notifier_.stop();
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
		_limelightUpdate.startPeriodic(GENERAL.kLimelightLoopPeriod);
		telemetry_notifier_.startPeriodic(kTelemetryPeriod);

		timestamp_ = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.onStop(timestamp_);
		}
	}

}
