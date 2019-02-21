package frc.robot.auto.actions;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ClimbState;

public class ActuateFrontSolenoids extends RunOnceAction {

    private ClimbState state;

    public ActuateFrontSolenoids(ClimbState state) {
        this.state = state;
    }

    @Override public void runOnce() {
        Superstructure.getInstance().setFrontClimbState(state);
    }
}
