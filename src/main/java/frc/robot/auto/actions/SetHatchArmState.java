package frc.robot.auto.actions;

import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchMechanismState;

public class SetHatchArmState extends RunOnceAction {

    private HatchMechanismState state;

    public SetHatchArmState(HatchMechanismState state) {
        this.state = state;
    }

    @Override public void runOnce() {
        HatchArm.getInstance().setHatchMechanismState(state);
    }
}
