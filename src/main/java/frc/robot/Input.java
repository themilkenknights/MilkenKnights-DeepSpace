package frc.robot;

import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;

public class Input {

    private static final MkJoystick driverJoystick = new MkJoystick(0);
    private static final MkJoystickButton changeDriveMode = driverJoystick.getButton(1, "Change Drive Mode");

    public static void updateDriveInput() {
        if (changeDriveMode.isPressed()) {
            Drive.getInstance().targetPos();
        }
        if (Drive.getInstance().mDriveControlState != DriveControlState.PATH_FOLLOWING) {
            double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
            double turn = (-driverJoystick.getRawAxis(0));
            Drive.getInstance().setOpenLoop(DriveHelper.cheesyDrive(forward, turn, true));
        }
    }

}
