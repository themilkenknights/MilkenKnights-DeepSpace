package frc.robot.lib.structure;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager {

	private final List<Subsystem> mAllSubsystems;
	private double lastTime, lastTime1, lastTime2 = 0.0;

	public SubsystemManager(List<Subsystem> allSubsystems) {
		mAllSubsystems = allSubsystems;
		lastTime = Timer.getFPGATimestamp();
		lastTime1 = Timer.getFPGATimestamp();
		lastTime2 = 0.0;
	}

	public void outputToSmartDashboard() {
		double timestamp_ = Timer.getFPGATimestamp();
		if (timestamp_ - lastTime >= 0.1) {
			mAllSubsystems.forEach((s) -> s.outputTelemetry());
			SmartDashboard.putNumber("Main loop Dt", (timestamp_ - lastTime2) * 1e3);
			lastTime = timestamp_;
		}
		lastTime2 = timestamp_;
	}

	public void onLoop() {
		double timestamp_ = Timer.getFPGATimestamp();
		double totalT = 0.0;
		if(false){
		//if(timestamp_ - lastTime1 >= 0.5){
			ArrayList<String> times = new ArrayList<String>();
			for (Subsystem subsystem : mAllSubsystems) {
				double stamp1 = Timer.getFPGATimestamp();
				subsystem.readPeriodicInputs(timestamp_);
				subsystem.onLoop(timestamp_);
				subsystem.writePeriodicOutputs(timestamp_);
				times.add(subsystem.getClass().getName() + ": " + (Timer.getFPGATimestamp() - stamp1) * 1e3);
				totalT += (Timer.getFPGATimestamp() - stamp1) * 1e3;
			}
				for(String ti : times){
					System.out.println(ti);
				}

				System.out.println("TOTAL TIME " + totalT);

			lastTime1 = timestamp_;
		} else{
			for (Subsystem subsystem : mAllSubsystems) {
				subsystem.readPeriodicInputs(timestamp_);
				subsystem.onLoop(timestamp_);
				subsystem.writePeriodicOutputs(timestamp_);
			}
		}


	}

	public void zero() {
		double timestamp_ = Timer.getFPGATimestamp();
		for (Subsystem subsystem : mAllSubsystems) {
			subsystem.zero(timestamp_);
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
}
