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
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;

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
	private static final MkJoystickButton killAuto = operatorJoystick.getButton(0, "Kill Auto");
	private static final MkJoystickButton intakeRollerIn = operatorJoystick.getButton(3, "Intake Roller In");
	private static final MkJoystickButton intakeRollerOutFast = operatorJoystick.getButton(9, "Intake Roller Out Fast");

	private static Drive mDrive = Drive.getInstance();
	private static HatchArm mHatch = HatchArm.getInstance();
	private static CargoArm mCargo = CargoArm.getInstance();
	private static Superstructure mStructure = Superstructure.getInstance();

	protected static void updateDriveInput() {
		RobotState currentRobotState = mStructure.getRobotState();
		if (Math.abs(driverJoystick.getRawAxis(0)) > 0.35 || killAuto.isPressed()) {
			mStructure.setRobotState(RobotState.TELEOP_DRIVE);
		}
		if (currentRobotState == RobotState.TELEOP_DRIVE) {
			double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
			double turn = (-driverJoystick.getRawAxis(0));
			mDrive.setOpenLoop(DriveHelper.cheesyDrive(forward, turn, true));
		} else if (visionIntakeHatch.isPressed()) {
			mStructure.setRobotState(RobotState.VISION_INTAKE_STATION);
		}

		if (armChangeModeButton.isPressed()) {
			mCargo.changeSafety();
		}
		if (mCargo.getArmControlState() == CargoArmControlState.MOTION_MAGIC) {
			if (armIntakeButton.isPressed()) {
				mCargo.setArmState(CargoArmState.INTAKE);
			}
		} else if (CargoArm.getInstance().getArmControlState() == CargoArmControlState.OPEN_LOOP) {
			mCargo.setOpenLoop(MkMath.handleDeadband(operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
		}

		if (stowButton.isPressed()) {
			mHatch.setHatchMechanismState(HatchMechanismState.STOWED);
		} else if (placeButton.isPressed()) {
			mHatch.setHatchMechanismState(HatchMechanismState.PLACING);
		} else if (stationIntakeButton.isPressed()) {
			mHatch.setHatchMechanismState(HatchMechanismState.STATION_INTAKE);
		} else if (grounIntakeButton.isPressed()) {
			mHatch.setHatchMechanismState(HatchMechanismState.GROUND_INTAKE);
		}

	}
}
