package frc.robot;

import frc.robot.Constants.INPUT;
import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.Logger;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.ArmControlState;
import frc.robot.subsystems.CargoArm.ArmState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchArmState;

public class Input {

  private static final MkJoystick driverJoystick = new MkJoystick(0);
  private static final MkJoystick operatorJoystick = new MkJoystick(1);
  private static final MkJoystickButton visionIntakeHatch = driverJoystick.getButton(3, "Vision Intake Hatch");
  private static final MkJoystickButton stowButton = driverJoystick.getButton(1, "Stow Button");
  private static final MkJoystickButton placeButton = driverJoystick.getButton(2, "Place Button");

  private static final MkJoystickButton armIntakeButton = operatorJoystick.getButton(2, "Arm Intake");
  private static final MkJoystickButton armDisableSafety = operatorJoystick.getButton(11, "Arm Disable Current Limit");
  private static final MkJoystickButton armSwitchButton = operatorJoystick.getButton(6, "Arm Forward Outtake");
  private static final MkJoystickButton armSwitchReverseButton = operatorJoystick.getButton(4, "Arm Reverse Outtake");
  private static final MkJoystickButton armChangeModeButton = operatorJoystick.getButton(8, "Arm Change Mode");
  private static final MkJoystickButton armZeroButton = operatorJoystick.getButton(7, "Arm Zero");
  private static final MkJoystickButton intakeRollerIn = operatorJoystick.getButton(3, "Intake Roller In");
  private static final MkJoystickButton intakeRollerOutFast = operatorJoystick.getButton(9, "Intake Roller Out Fast");

  public static void updateDriveInput() {

    switch (CargoArm.mArmControlState) {
      case MOTION_MAGIC:
        if (armIntakeButton.isPressed()) {
          CargoArm.mArmState = ArmState.INTAKE;
        } else if (armSwitchButton.isPressed()) {
          CargoArm.mArmState = ArmState.SWITCH_PLACE;
        } else if (armSwitchReverseButton.isPressed()) {
          CargoArm.mArmState = ArmState.OPPOSITE_STOW;
        } else if (operatorJoystick.getPOV() != -1) {
          CargoArm.mArmState = ArmState.OPPOSITE_SWITCH_PLACE;
        }
        if (armChangeModeButton.isPressed()) {
          CargoArm.mArmControlState = ArmControlState.OPEN_LOOP;
        }
        break;
      case OPEN_LOOP:
        CargoArm.getInstance().setOpenLoop(MkMath
            .handleDeadband(operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
        if (armChangeModeButton.isPressed()) {
          CargoArm.getInstance().setEnable();
          CargoArm.mArmControlState = ArmControlState.MOTION_MAGIC;
        }
        break;
      default:
        Logger.logMarker("Unexpected Arm control state: " + CargoArm.mArmControlState);
        break;
    }

    if (stowButton.isPressed()) {
      HatchArm.getInstance().setHatchArm(HatchArmState.STOW);
    } else if (placeButton.isPressed()) {
      HatchArm.getInstance().setHatchArm(HatchArmState.PLACE);
    }
    if (Drive.getInstance().mDriveControlState != DriveControlState.PATH_FOLLOWING) {
      double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
      double turn = (-driverJoystick.getRawAxis(0));
      Drive.getInstance().setOpenLoop(DriveHelper.cheesyDrive(forward, turn, true));
    }
  }
}
