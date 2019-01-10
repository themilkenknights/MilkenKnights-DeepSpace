package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.structure.ILooper;
import frc.robot.lib.structure.Loop;

public class Superstructure extends Subsystem {

	public static Superstructure getInstance() {
		return InstanceHolder.mInstance;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("Robot State", Robot.mMatchState.toString());
	}


	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {

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
		});
	}

	private static class InstanceHolder {

		private static final Superstructure mInstance = new Superstructure();

	}
}
