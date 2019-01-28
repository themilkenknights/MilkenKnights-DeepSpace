package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.CollectAccelerationData;
import frc.robot.auto.actions.CollectVelocityData;
import frc.robot.auto.actions.WaitAction;
import frc.robot.lib.physics.DriveCharacterization;

import java.util.ArrayList;
import java.util.List;

public class CharacterizeHighGearStraight extends AutoModeBase {
		@Override
		protected void routine() throws AutoModeEndedException {
				List<DriveCharacterization.VelocityDataPoint> velocityData = new ArrayList<>();
				List<DriveCharacterization.AccelerationDataPoint> accelerationData = new ArrayList<>();
				runAction(new WaitAction(1));
				runAction(new CollectVelocityData(velocityData, false, false));
				runAction(new WaitAction(3));
				runAction(new CollectAccelerationData(accelerationData, false, false));
				DriveCharacterization.CharacterizationConstants constants = DriveCharacterization.characterizeDrive(velocityData, accelerationData);
				System.out.println("ks: " + constants.ks);
				System.out.println("kv: " + constants.kv);
				System.out.println("ka: " + constants.ka);
		}
}
