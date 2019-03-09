package frc.robot.auto.actions;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ClimbState;

public class ActuateRearSolenoids extends RunOnceAction {

	private ClimbState state;

	public ActuateRearSolenoids(ClimbState state) {
		this.state = state;
	}

	@Override
	public void runOnce() {
		Superstructure.getInstance().setRearClimbState(state);
	}
}
