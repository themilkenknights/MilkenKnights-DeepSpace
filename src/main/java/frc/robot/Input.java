package frc.robot;

import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchArmState;

public class Input {

  private static final MkJoystick driverJoystick = new MkJoystick(0);
  private static final MkJoystickButton visionIntakeHatch = driverJoystick.getButton(3, "Vision Intake Hatch");
  private static final MkJoystickButton stowButton = driverJoystick.getButton(1, "Stow Button");
  private static final MkJoystickButton placeButton = driverJoystick.getButton(2, "Place Button");

  public static void updateDriveInput() {
    if(visionIntakeHatch.isPressed()){

    }
    if (stowButton.isPressed()) {
      HatchArm.getInstance().setHatchArm(HatchArmState.STOW);
    } else if (placeButton.isPressed()){
      HatchArm.getInstance().setHatchArm(HatchArmState.PLACE);
    }
    if (Drive.getInstance().mDriveControlState != DriveControlState.PATH_FOLLOWING) {
      double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
      double turn = (-driverJoystick.getRawAxis(0));
      Drive.getInstance().setOpenLoop(DriveHelper.cheesyDrive(forward, turn, true));
    }
  }
}
