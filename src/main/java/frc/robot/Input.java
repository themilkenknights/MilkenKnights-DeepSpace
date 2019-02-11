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
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.Vision;

public class Input {

	private static final MkJoystick driverJoystick = new MkJoystick(0);
	private static final MkJoystick operatorJoystick = new MkJoystick(1);

	private static final MkJoystickButton toggleDriverVisionAssist = driverJoystick.getButton(4, "Driver Vision Assist");

	private static final MkJoystickButton mVisionStationIntakeButton = operatorJoystick.getButton(3, "Vision Hatch HP Intake Button");
	private static final MkJoystickButton mVisionPlaceButton = operatorJoystick.getButton(4, "Vision Place Hatch Button");
	private static final MkJoystickButton mGroundIntakeToggleButton = operatorJoystick.getButton(11, "Ground Intake Toggle Button");
	private static final MkJoystickButton mTransferButton = operatorJoystick.getButton(12, "Transfer Hatch Button");

	private static final MkJoystickButton mGroundHatchIntakeManual = operatorJoystick.getButton(5, "Ground Hatch Intake Manual Mode");
	private static final MkJoystickButton mCargoArmManual = operatorJoystick.getButton(2, "Cargo Arm Manual Mode");

	//private static final MkJoystickButton killAuto = operatorJoystick.getButton(1, "Kill Auto");

	private static final MkJoystickButton intakeRollerIn = operatorJoystick.getButton(5, "Intake Roller In");
	private static final MkJoystickButton intakeRollerOut = operatorJoystick.getButton(6, "Intake Roller Out Fast");

	private static Drive mDrive = Drive.getInstance();
	private static HatchArm mHatch = HatchArm.getInstance();
	private static CargoArm mCargo = CargoArm.getInstance();
	private static Superstructure mStructure = Superstructure.getInstance();
	private static Vision mVision = Vision.getInstance();


	private static boolean mVisionAssist = false;

	public static void updateControlInput() {
		RobotState currentRobotState = mStructure.getRobotState();

		boolean isVisionState =
				currentRobotState == RobotState.VISION_CARGO_INTAKE
						|| currentRobotState == RobotState.VISION_CARGO_OUTTAKE
						|| currentRobotState == RobotState.VISION_INTAKE_STATION
						|| currentRobotState == RobotState.VISION_PLACING;

		if ((Math.abs(driverJoystick.getRawAxis(0)) > 0.35) && isVisionState) {
			mStructure.setRobotState(RobotState.TELEOP_DRIVE);
		}

		if (mCargoArmManual.isPressed()) {
			mCargo.enableSafety(!mCargo.getSafetyState());

		} else if (mGroundHatchIntakeManual.isPressed()) {
			mHatch.setHatchMechanismState(
					mHatch.getHatchMechanismState() == HatchMechanismState.MANUAL_OVERRIDE ? HatchMechanismState.UNKNOWN : HatchMechanismState.MANUAL_OVERRIDE);

		}

		if (toggleDriverVisionAssist.isPressed()) {
			mVisionAssist = !mVisionAssist;
		}

		if (currentRobotState == RobotState.TELEOP_DRIVE) {
			double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
			double turn = (-driverJoystick.getRawAxis(0));
			LimelightTarget target = mVision.getAverageTarget();
			double mSteer = target.getDistance() < 50.0 && target.isValidTarget() && mVisionAssist ? DRIVE.kVisionDriverTurnP * target.getXOffset() : 0.0;
			mDrive.setOpenLoop(DriveHelper.cheesyDrive(forward, turn + mSteer, true));
		}

		if (mCargo.getArmControlState() == CargoArmControlState.OPEN_LOOP) {
			mCargo.setOpenLoop(MkMath.handleDeadband(operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
		} else if (mCargo.getArmControlState() == CargoArmControlState.MOTION_MAGIC) {
			if (operatorJoystick.getPOV() == 0) {
				mCargo.setArmState(CargoArmState.PLACE_REVERSE_ROCKET);

			} else if (operatorJoystick.getPOV() == 270) {
				mCargo.setArmState(CargoArmState.INTAKE);
			} else if (operatorJoystick.getPOV() == 180) {
				mCargo.setArmState(CargoArmState.PLACE_REVERSE_CARGO);

			} else if (operatorJoystick.getPOV() == 90) {

				mStructure.setRobotState(RobotState.VISION_CARGO_OUTTAKE);

			}
		}

		if (intakeRollerIn.isHeld()) {
			mCargo.setIntakeRollers(CARGO_ARM.INTAKE_IN_ROLLER_SPEED);

		} else if (intakeRollerOut.isHeld()) {
			mCargo.setIntakeRollers(CARGO_ARM.INTAKE_OUT_ROLLER_SPEED);

		} else {
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
		}

		if (mVisionStationIntakeButton.isPressed()) {

			mStructure.setRobotState(RobotState.VISION_INTAKE_STATION);
		} else if (mVisionPlaceButton.isPressed()) {

			mStructure.setRobotState(RobotState.VISION_PLACING);
		} else if (mTransferButton.isPressed()) {

			mHatch.setHatchMechanismState(HatchMechanismState.TRANSFER);

		} else if (mGroundIntakeToggleButton.isPressed()) {

			mHatch
					.setHatchMechanismState(mHatch.getHatchMechanismState() == HatchMechanismState.GROUND_INTAKE ? HatchMechanismState.STOWED : HatchMechanismState.GROUND_INTAKE);

		}


	}
}
