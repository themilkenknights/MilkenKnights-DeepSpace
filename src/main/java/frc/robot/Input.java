package frc.robot;

import frc.robot.Constants.INPUT;
import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.lib.math.MkMath;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.CargoArmControlState;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchMechanismState;

public class Input {

  private static final MkJoystick driverJoystick = new MkJoystick(0);
  private static final MkJoystick operatorJoystick = new MkJoystick(1);
  private static final MkJoystickButton visionIntakeHatch = driverJoystick.getButton(3, "Vision Intake Hatch");

  private static final MkJoystickButton stowButton = driverJoystick.getButton(1, "Stow Button");
  private static final MkJoystickButton placeButton = driverJoystick.getButton(2, "Place Button");
  private static final MkJoystickButton stationIntakeButton = driverJoystick.getButton(3, "Station Intake");
  private static final MkJoystickButton grounIntakeButton = driverJoystick.getButton(4, "Ground Intake");

  private static final MkJoystickButton armIntakeButton = operatorJoystick.getButton(2, "Arm Intake");
  private static final MkJoystickButton armChangeModeButton = operatorJoystick.getButton(8, "Arm Change Mode");
  private static final MkJoystickButton intakeRollerIn = operatorJoystick.getButton(3, "Intake Roller In");
  private static final MkJoystickButton intakeRollerOutFast = operatorJoystick.getButton(9, "Intake Roller Out Fast");

  public static void updateDriveInput() {
    if (armChangeModeButton.isPressed()) {
      CargoArm.getInstance().changeSafety();
    }
    if (CargoArm.getInstance().getArmControlState() == CargoArmControlState.MOTION_MAGIC) {
      if (operatorJoystick.getPOV() != -1) {
        CargoArm.getInstance().setArmState(CargoArmState.OPPOSITE_SWITCH_PLACE);
      }
    } else if (CargoArm.getInstance().getArmControlState() == CargoArmControlState.OPEN_LOOP) {
      CargoArm.getInstance().setOpenLoop(MkMath.handleDeadband(operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
    }

    if (stowButton.isPressed()) {
      HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.STOWED);
    } else if (placeButton.isPressed()) {
      HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.PLACING);
    } else if (stationIntakeButton.isPressed()) {
      HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.STATION_INTAKE);
    } else if (grounIntakeButton.isPressed()) {
      HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.GROUND_INTAKE);
    }

    if (visionIntakeHatch.isPressed()) {
      Drive.getInstance().startVisionTracking();
    } else if (Drive.getInstance().mDriveControlState != DriveControlState.PATH_FOLLOWING) {
      double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
      double turn = (-driverJoystick.getRawAxis(0));
      Drive.getInstance().setOpenLoop(DriveHelper.cheesyDrive(forward, turn, true));
    }
  }
}
