package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.structure.ILooper;
import frc.robot.lib.structure.Loop;

import java.util.ArrayList;
import java.util.List;

/**
 * Used to reset, start, stop, and update all subsystems at once
 */
public class SubsystemManager implements ILooper {
		private final List<Subsystem> mAllSubsystems;
		private List<Loop> mLoops = new ArrayList<>();

		public SubsystemManager(List<Subsystem> allSubsystems) {
				mAllSubsystems = allSubsystems;
		}

		public void outputToSmartDashboard() {
				mAllSubsystems.forEach((s) -> s.outputTelemetry());
		}

		@Override
		public void register(Loop loop) {
				mLoops.add(loop);
		}

		public void onLoop(){
				double timestamp_ = Timer.getFPGATimestamp();
				for (Loop loop : mLoops) {
						loop.onLoop(timestamp_);
				}
		}

		public void start(){
				double timestamp_ = Timer.getFPGATimestamp();
				for (Loop loop : mLoops) {
						loop.onStart(timestamp_);
				}
		}

		public void stop(){
				double timestamp_ = Timer.getFPGATimestamp();
				for (Loop loop : mLoops) {
						loop.onStop(timestamp_);
				}
		}


}
