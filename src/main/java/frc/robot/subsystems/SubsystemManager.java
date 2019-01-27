package frc.robot.subsystems;

import frc.robot.lib.structure.ILooper;
import frc.robot.lib.structure.Loop;
import frc.robot.lib.structure.Looper;

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

		public void registerEnabledLoops(Looper enabledLooper) {
				mAllSubsystems.forEach((s) -> s.registerEnabledLoops(this));
				enabledLooper.register(new EnabledLoop());
		}

		public void registerDisabledLoops(Looper disabledLooper) {
				disabledLooper.register(new DisabledLoop());
		}

		@Override public void register(Loop loop) {
				mLoops.add(loop);
		}

		private class EnabledLoop implements Loop {
				@Override public void onStart(double timestamp) {
						for (Loop l : mLoops) {
								l.onStart(timestamp);
						}
				}

				@Override public void onLoop(double timestamp) {
						for (Subsystem s : mAllSubsystems) {
								s.readPeriodicInputs();
						}
						for (Loop l : mLoops) {
								l.onLoop(timestamp);
						}
						for (Subsystem s : mAllSubsystems) {
								s.writePeriodicOutputs();
						}
				}

				@Override public void onStop(double timestamp) {
						for (Loop l : mLoops) {
								l.onStop(timestamp);
						}
				}
		}


		private class DisabledLoop implements Loop {
				@Override public void onStart(double timestamp) {
				}

				@Override public void onLoop(double timestamp) {
						for (Subsystem s : mAllSubsystems) {
								s.readPeriodicInputs();
						}
						for (Subsystem s : mAllSubsystems) {
								s.writePeriodicOutputs();
						}
				}

				@Override public void onStop(double timestamp) {
				}
		}
}
