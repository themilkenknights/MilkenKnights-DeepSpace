package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ControlState;
import frc.robot.lib.structure.Loop;
import frc.robot.lib.structure.Looper;

public class Superstructure extends Subsystem {

	public Superstructure() {

	}

	public static Superstructure getInstance() {
		return InstanceHolder.mInstance;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("Robot State", ControlState.mMatchState.toString());
	}


	@Override
	public void checkSystem() {

	}


	@Override
	public void stop() {

	}

	public void registerEnabledLoops(Looper enabledLooper) {
		Loop mLoop = new Loop() {

			@Override
			public void onStart(double timestamp) {
				synchronized (Superstructure.this) {
				}
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Superstructure.this) {
				}
			}

			@Override
			public void onStop(double timestamp) {

			}
		};
		enabledLooper.register(mLoop);
	}

	private static class InstanceHolder {

		private static final Superstructure mInstance = new Superstructure();

	}
}
