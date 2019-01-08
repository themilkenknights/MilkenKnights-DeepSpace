package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.util.structure.Subsystem;
import frc.robot.util.structure.loops.Loop;
import frc.robot.util.structure.loops.Looper;

public class Superstructure extends Subsystem {

	public Superstructure() {

	}

	public static Superstructure getInstance() {
		return InstanceHolder.mInstance;
	}

	@Override
	public void outputToSmartDashboard() {
		SmartDashboard.putString("Robot State", RobotState.mMatchState.toString());
	}

	@Override
	public void slowUpdate(double timestamp) {

	}

	@Override
	public void checkSystem() {

	}

	@Override
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
