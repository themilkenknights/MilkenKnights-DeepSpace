package frc.robot.auto.modes;

import frc.robot.AutoChooser;
import frc.robot.auto.actions.DrivePathAction;
import frc.robot.util.auto.AutoModeBase;
import frc.robot.util.auto.AutoModeEndedException;

public class DriveStraightMode extends AutoModeBase {

    @Override
    protected void routine() throws AutoModeEndedException {

        runAction(new DrivePathAction(AutoChooser.autoPaths.get("DriveStraightLB"), false, false, false));

    }
}
