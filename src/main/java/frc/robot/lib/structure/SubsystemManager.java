package frc.robot.lib.structure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager {

	private final List<Subsystem> mAllSubsystems;
	private double lastTime, lastTime2;

	public SubsystemManager(List<Subsystem> allSubsystems) {
		mAllSubsystems = allSubsystems;
		lastTime = Timer.getFPGATimestamp();
		lastTime2 = 0.0;
	}

	public void outputToSmartDashboard() {
		double timestamp_ = Timer.getFPGATimestamp();
		if (timestamp_ - lastTime >= 0.25) {
			mAllSubsystems.forEach((s) -> s.outputTelemetry());
			SmartDashboard.putNumber("Main loop Dt", (timestamp_ - lastTime2) * 1e3);
			lastTime = timestamp_;
		}
		lastTime2 = timestamp_;
	}

	public void onLoop() {
		double timestamp_ = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.readPeriodicInputs(timestamp_);
			subsystem.onLoop(timestamp_);
			subsystem.writePeriodicOutputs(timestamp_);
		}
	}

	public void stop() {
		double timestamp_ = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.onStop(timestamp_);
		}
	}

	public void checkSystem() {
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.checkSystem();
		}
	}

	public void teleopInit() {
		double timestamp_ = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.teleopInit(timestamp_);
		}
	}

	public void autonomousInit() {
		double timestamp_ = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.autonomousInit(timestamp_);
		}
	}
}
