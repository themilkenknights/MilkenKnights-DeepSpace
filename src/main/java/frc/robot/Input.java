package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.CARGO_ARM;
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
import frc.robot.subsystems.HatchArm.HatchArmState;
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;

public class Input {

	private static final MkJoystick driverJoystick = new MkJoystick(0);
	private static final MkJoystick operatorJoystick = new MkJoystick(1);

	private static final MkJoystickButton visionIntakeHatch = driverJoystick.getButton(13, "Vision Intake Hatch");

	private static final MkJoystickButton mStationIntakeButton = driverJoystick.getButton(14, "Station Intake Button");
	private static final MkJoystickButton mGroundIntakeButton = driverJoystick.getButton(15, "Ground Intake Button");
	private static final MkJoystickButton mPlaceButton = driverJoystick.getButton(16, "Place Hatch Button");
	private static final MkJoystickButton mTransferButton = driverJoystick.getButton(17, "Transfer Hatch Button");

	private static final MkJoystickButton mGroundHatchIntakeManual = operatorJoystick.getButton(1, "Ground Hatch Intake Manual Mode");
	private static final MkJoystickButton mCargoArmManual =operatorJoystick.getButton(2, "Cargo Arm Manual Mode");

	private static final MkJoystickButton armIntakeButton = operatorJoystick.getButton(3, "Arm Intake");
	private static final MkJoystickButton killAuto = operatorJoystick.getButton(4, "Kill Auto");
	private static final MkJoystickButton intakeRollerIn = operatorJoystick.getButton(5, "Intake Roller In");
	private static final MkJoystickButton intakeRollerOutFast = operatorJoystick.getButton(6, "Intake Roller Out Fast");

	private static Drive mDrive = Drive.getInstance();
	private static HatchArm mHatch = HatchArm.getInstance();
	private static CargoArm mCargo = CargoArm.getInstance();
	private static Superstructure mStructure = Superstructure.getInstance();

	public static void updateDriveInput() {
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

		if (mCargoArmManual.isPressed()) {
			mCargo.changeSafety();
		}

		if (mGroundHatchIntakeManual.isPressed()) {
			mHatch.changeSafety(mHatch.getHatchMechanismState() != HatchMechanismState.MANUAL_OVERRIDE);
			System.out.println("Change");
		}

		if (mCargo.getArmControlState() == CargoArmControlState.OPEN_LOOP) {
			mCargo.setOpenLoop(MkMath.handleDeadband(operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
		} else if (mCargo.getArmControlState() == CargoArmControlState.MOTION_MAGIC) {
			if (armIntakeButton.isPressed()) {
				mCargo.setArmState(CargoArmState.INTAKE);
			}
		}

		if (intakeRollerIn.isHeld()) {
			mCargo.setIntakeRollers(CARGO_ARM.INTAKE_IN_ROLLER_SPEED);
		} else if (intakeRollerOutFast.isHeld()) {
			mCargo.setIntakeRollers(CARGO_ARM.INTAKE_OUT_ROLLER_SPEED);
		}

		//If in manual override mode, use open loop control for Arm (except when both arms are in manual mode)
		//The normal hatch buttons now only serve to actuate the pneumatic arm
		if (mHatch.getHatchMechanismState() == HatchMechanismState.MANUAL_OVERRIDE) {

			if (mCargo.getArmControlState() != CargoArmControlState.OPEN_LOOP) {
				mCargo.setOpenLoop(MkMath.handleDeadband(operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
			} else {
				mHatch.setOpenLoop(0.0);
			}

			if (mStationIntakeButton.isPressed() || mPlaceButton.isPressed()) {
				mHatch.setHatchArmPosition(HatchArmState.PLACE);
			} else if (mTransferButton.isPressed() || mGroundIntakeButton.isPressed()) {
				mHatch.setHatchArmPosition(HatchArmState.STOW);
			}

		} else {
			if (mStationIntakeButton.isPressed()) {
				mHatch.setHatchMechanismState(HatchMechanismState.STATION_INTAKE);
			} else if (mPlaceButton.isPressed()) {
				mHatch.setHatchMechanismState(HatchMechanismState.PLACING);
			} else if (mTransferButton.isPressed()) {
				mHatch.setHatchMechanismState(HatchMechanismState.TRANSFER);
			} else if (mGroundIntakeButton.isPressed()) {
				mHatch.setHatchMechanismState(HatchMechanismState.GROUND_INTAKE);
			}
		}


	}
}
