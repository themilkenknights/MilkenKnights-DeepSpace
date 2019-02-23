package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CollectAccelerationData;
import frc.robot.auto.actions.CollectVelocityData;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.physics.DriveCharacterization;
import frc.robot.lib.util.Logger;
import java.util.ArrayList;
import java.util.List;

public class CharacterizeHighGearStraight extends AutoModeBase {

    @Override protected void routine() throws AutoModeEndedException {
        List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
        List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();
        runAction(new WaitAction(1));
        runAction(new CollectVelocityData(velocityData, false, true));
        runAction(new WaitAction(2));
        runAction(new CollectAccelerationData(accelerationData, false, true));
        DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);
        Logger.logMarker("ks: " + constants.ks);
        Logger.logMarker("kv: " + constants.kv);
        Logger.logMarker("ka: " + constants.ka);
        Logger.logMarker("DONEEEEE");
    }
}
