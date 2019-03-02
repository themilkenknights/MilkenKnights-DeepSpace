package frc.robot.auto.modes;

import frc.robot.Constants.CARGO_ARM;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.MotionMagicVisionCargo;
import frc.robot.auto.actions.RollerAction;

public class SimpleCargoOuttake extends AutoModeBase {

    @Override protected void routine() throws AutoModeEndedException {
        runAction(new MotionMagicVisionCargo());
        runAction(new RollerAction(CARGO_ARM.kCargoShipIntakeRollerOut, CARGO_ARM.kCargoShipIntakeRollerOut));
    }
}
