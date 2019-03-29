package frc.robot.auto.actions;

import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class ChangeMotionMagicAccel extends RunOnceAction {
  @Override public void runOnce() {
    Drive.getInstance().changeMotionMagicAccel((int) (Constants.DRIVE.kMaxNativeVel * 0.4));
  }
}
