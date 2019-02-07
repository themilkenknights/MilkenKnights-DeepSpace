package frc.robot;

import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.INPUT;
import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.CargoArmControlState;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchArmState;
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.Vision;

public class Input {

	private static final MkJoystick driverJoystick = new MkJoystick(0);
	private static final MkJoystick operatorJoystick = new MkJoystick(1);

	private static final MkJoystickButton toggleDriverVisionAssist = driverJoystick.getButton(4, "Driver Vision Assist");
	private static final MkJoystickButton visionIntakeHatch = driverJoystick.getButton(3, "Vision Intake Hatch");

	private static final MkJoystickButton mStationIntakeButton = operatorJoystick.getButton(3, "Station Intake Button");
	private static final MkJoystickButton mGroundIntakeButton = operatorJoystick.getButton(2, "Ground Intake Button");
	private static final MkJoystickButton mPlaceButton = operatorJoystick.getButton(4, "Place Hatch Button");
	private static final MkJoystickButton mTransferButton = operatorJoystick.getButton(1, "Transfer Hatch Button");

	private static final MkJoystickButton mGroundHatchIntakeManual = operatorJoystick.getButton(5, "Ground Hatch Intake Manual Mode");
	private static final MkJoystickButton mCargoArmManual = operatorJoystick.getButton(6, "Cargo Arm Manual Mode");

	private static final MkJoystickButton mCargoArmIntakeButton = operatorJoystick.getButton(11, "Arm Intake");
	private static final MkJoystickButton mCargoArmPlaceButton = operatorJoystick.getButton(12, "Arm Place");
	private static final MkJoystickButton mCargoArmStowButton = operatorJoystick.getButton(9, "Arm Stow");
	private static final MkJoystickButton mCargoArmReversePlaceButton = operatorJoystick.getButton(10, "Arm Reverse");

	//private static final MkJoystickButton killAuto = operatorJoystick.getButton(1, "Kill Auto");

	private static final MkJoystickButton intakeRollerIn = operatorJoystick.getButton(7, "Intake Roller In");
	private static final MkJoystickButton intakeRollerOutFast = operatorJoystick.getButton(8, "Intake Roller Out Fast");
	//TODO Fix
	private static Drive mDrive = Drive.getInstance();
	private static HatchArm mHatch = HatchArm.getInstance();
	private static CargoArm mCargo = CargoArm.getInstance();
	private static Superstructure mStructure = Superstructure.getInstance();

	private static boolean mVisionAssist = false;

	public static void updateDriveInput() {
		RobotState currentRobotState = mStructure.getRobotState();
		if ((Math.abs(driverJoystick.getRawAxis(0)) > 0.35 || operatorJoystick.getPOV() != -1) && (currentRobotState == RobotState.VISION_CARGO_INTAKE
				|| currentRobotState == RobotState.VISION_CARGO_OUTTAKE || currentRobotState == RobotState.VISION_INTAKE_STATION
				|| currentRobotState == RobotState.VISION_PLACING)) {
			mStructure.setRobotState(RobotState.TELEOP_DRIVE);
		}

		mVisionAssist = toggleDriverVisionAssist.isPressed() != mVisionAssist;
		if (currentRobotState == RobotState.TELEOP_DRIVE) {
			double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
			double turn = (-driverJoystick.getRawAxis(0));
			LimelightTarget target = Vision.getInstance().getAverageTarget();
			double mSteer = target.getDistance() < 50.0 && target.isValidTarget() && mVisionAssist
					? DRIVE.kVisionDriverTurnP * target.getXOffset()
					: 0.0;
			mDrive.setOpenLoop(DriveHelper.cheesyDrive(forward, turn + mSteer, true));
		} else if (visionIntakeHatch.isPressed()) {
			mStructure.setRobotState(RobotState.VISION_INTAKE_STATION);
		}

		if (mCargoArmManual.isPressed()) {
			mCargo.changeSafety();
		}

		if (mGroundHatchIntakeManual.isPressed()) {
			mHatch.setHatchMechanismState(mHatch.getHatchMechanismState() == HatchMechanismState.MANUAL_OVERRIDE ? HatchMechanismState.UNKNOWN : HatchMechanismState.MANUAL_OVERRIDE);
			System.out.println("Change");
		}

		if (mCargo.getArmControlState() == CargoArmControlState.OPEN_LOOP) {
			mCargo.setOpenLoop(MkMath.handleDeadband(operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
		} else if (mCargo.getArmControlState() == CargoArmControlState.MOTION_MAGIC) {
			if (mCargoArmStowButton.isPressed()) {
				mCargo.setArmState(CargoArmState.STOW);
			} else if (mCargoArmIntakeButton.isPressed()) {
				mCargo.setArmState(CargoArmState.INTAKE);
			} else if (mCargoArmPlaceButton.isPressed()) {
				mCargo.setArmState(CargoArmState.PLACE);
			} else if (mCargoArmReversePlaceButton.isPressed()) {
				mCargo.setArmState(CargoArmState.PLACE_REVERSE);
			}
		}

		if (intakeRollerIn.isHeld()) {
			mCargo.setIntakeRollers(CARGO_ARM.INTAKE_IN_ROLLER_SPEED);
		} else if (intakeRollerOutFast.isHeld()) {
			mCargo.setIntakeRollers(CARGO_ARM.INTAKE_OUT_ROLLER_SPEED);
		} else{
			mCargo.setIntakeRollers(0.0);
		}

		// If in manual override mode, use open loop control for Arm (except when both
		// arms are in manual mode)
		// The normal hatch buttons now only serve to actuate the pneumatic arm
		if (mHatch.getHatchMechanismState() == HatchMechanismState.MANUAL_OVERRIDE) {

			if (mCargo.getArmControlState() != CargoArmControlState.OPEN_LOOP) {
				mHatch.setOpenLoop(MkMath.handleDeadband(operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
			} else {
				mHatch.setOpenLoop(0.0);
			}

			if (mStationIntakeButton.isPressed()){
				mHatch.setHatchMechanismState(HatchMechanismState.STATION_INTAKE);
			} else if(mPlaceButton.isPressed()) {
				mHatch.setHatchMechanismState(HatchMechanismState.PLACING);
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
