package frc.robot;

import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.INPUT;
import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.CargoArmControlState;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchIntakeControlState;
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import frc.robot.subsystems.HatchArm.HatchSpearState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ClimbState;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.Vision;

public class Input {

    private static final MkJoystick driverJoystick = new MkJoystick(0);
    private static final MkJoystick operatorJoystick = new MkJoystick(1);

    private static final MkJoystickButton toggleDriverVisionAssist = driverJoystick.getButton(4, "Driver Vision Assist");
    private static final MkJoystickButton mRearClimb = driverJoystick.getButton(2, "Climb Rear");
    private static final MkJoystickButton mForwardClimb = driverJoystick.getButton(3, "Climb Climb");

    private static final MkJoystickButton mVisionStationIntakeButton = operatorJoystick.getButton(3, "Vision Hatch HP Intake");
    private static final MkJoystickButton mVisionPlaceButton = operatorJoystick.getButton(4, "Vision Place Hatch Button");

    private static final MkJoystickButton mCargoArmManual = operatorJoystick.getButton(2, "Cargo Arm Manual Mode");

    private static final MkJoystickButton mIntakeRollerIn = operatorJoystick.getButton(5, "Intake Roller In");
    private static final MkJoystickButton mIntakeRollerOut = operatorJoystick.getButton(6, "Intake Roller Out Fast");

    private static final MkJoystickButton mStowAllButton = operatorJoystick.getButton(7, "Stow All");

    private static final MkJoystickButton mSpearTogglePlaceStow = operatorJoystick.getButton(11, "Hatch Spear Toggle (Place/Stow)");
    private static final MkJoystickButton mSpearIntake = operatorJoystick.getButton(12, "Hatch Spear HP Intake");

    private static final MkJoystickButton mGroundIntakeToggleButton = operatorJoystick.getButton(9, "Ground Intake Toggle (Stow/Ground)");
    private static final MkJoystickButton mTransferButton = operatorJoystick.getButton(10, "Transfer Hatch Button");

    private static final MkJoystickButton mCargoRocketLevelTwo = driverJoystick.getButton(8, "Cargo Arm Rocket Level Two");


    private static Drive mDrive = Drive.getInstance();
    private static HatchArm mHatch = HatchArm.getInstance();
    private static CargoArm mCargo = CargoArm.getInstance();
    private static Superstructure mStructure = Superstructure.getInstance();
    private static Vision mVision = Vision.getInstance();

    private static boolean mVisionAssist = false;

    public static void updateControlInput() {
        RobotState currentRobotState = mStructure.getRobotState();
        boolean isVisionState = currentRobotState == RobotState.VISION_CARGO_INTAKE || currentRobotState == RobotState.VISION_CARGO_OUTTAKE
            || currentRobotState == RobotState.VISION_INTAKE_STATION || currentRobotState == RobotState.VISION_PLACING;

        if (isVisionState && (Math.abs(driverJoystick.getRawAxis(0)) > 0.35)) {
            mStructure.setRobotState(RobotState.TELEOP_DRIVE);
        } else if (toggleDriverVisionAssist.isPressed()) {
            mVisionAssist = !mVisionAssist;
        }

        if (mCargoArmManual.isPressed()) {
            mCargo.enableSafety(!mCargo.getSafetyState());
        } else if (operatorJoystick.getTriggerPressed()) {
            mHatch.enableSafety(mHatch.getHatchIntakeControlState() == HatchIntakeControlState.MOTION_MAGIC);
        }

        currentRobotState = mStructure.getRobotState();

        if (currentRobotState == RobotState.TELEOP_DRIVE) {
            double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
            double turn = (-driverJoystick.getRawAxis(0));
            LimelightTarget target = mVision.getLimelightTarget();
            double mSteer = target.getArea() > 1000 && target.isValidTarget() && mVisionAssist ? DRIVE.kVisionDriverTurnP * target.getXOffset() : 0.0;
            DriveSignal controlSig = DriveHelper.cheesyDrive(forward, turn, true);
            mDrive.setOpenLoop(controlSig);
        }

        if (mForwardClimb.isPressed()) {
            mStructure.setFrontClimbState(mStructure.getFrontClimbState() == ClimbState.RETRACTED ? ClimbState.LOWERED : ClimbState.RETRACTED);
        } else if (mRearClimb.isPressed()) {
            mStructure.setRearClimbState(mStructure.getRearClimbState() == ClimbState.RETRACTED ? ClimbState.LOWERED : ClimbState.RETRACTED);
        }

        if (mCargo.getArmControlState() == CargoArmControlState.OPEN_LOOP) {
            mCargo.setOpenLoop(MkMath.handleDeadband(-operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
        } else if (mCargo.getArmControlState() == CargoArmControlState.MOTION_MAGIC) {
            if (operatorJoystick.getPOV() == 0) {
                mCargo.setArmState(CargoArmState.FORWARD_ROCKET_LEVEL_ONE);
            } else if (operatorJoystick.getPOV() == 270) {
                mCargo.setArmState(CargoArmState.INTAKE);
            } else if (operatorJoystick.getPOV() == 180) {
                mCargo.setArmState(CargoArmState.REVERSE_CARGOSHIP);
            } else if (mCargoRocketLevelTwo.isPressed()) {
                mCargo.setArmState(CargoArmState.REVERSE_ROCKET_LEVEL_TWO);
            } else if (operatorJoystick.getPOV() == 90) {
                if (mCargo.getArmState() == CargoArmState.REVERSE_CARGOSHIP || mCargo.getArmState() == CargoArmState.FORWARD_ROCKET_LEVEL_ONE) {
                    mStructure.setRobotState(RobotState.VISION_CARGO_OUTTAKE);
                } else if (mCargo.getArmState() == CargoArmState.INTAKE) {
                    mStructure.setRobotState(RobotState.VISION_CARGO_INTAKE);
                } else {
                    Logger.logError("Failed to run Cargo Auto Mode");
                }
            }
        }

        if (mIntakeRollerIn.isHeld()) {
            mCargo.setIntakeRollers(CARGO_ARM.INTAKE_IN_ROLLER_SPEED);
        } else if (mIntakeRollerOut.isHeld()) {
            switch (mCargo.getArmState()) {
                case INTAKE:
                case ENABLE:
                    mCargo.setIntakeRollers(CARGO_ARM.DEFAULT_INTAKE_ROLLER_OUT_SPEED);
                    break;
                case FORWARD_ROCKET_LEVEL_ONE:
                    mCargo.setIntakeRollers(CARGO_ARM.ROCKET_LEVEL_ONE_INTAKE_OUT_ROLLER_SPEED);
                    break;
                case REVERSE_ROCKET_LEVEL_TWO:
                    mCargo.setIntakeRollers(CARGO_ARM.ROCKET_LEVEL_TWO_OUT_ROLLER_SPEED);
                    break;
                case REVERSE_CARGOSHIP:
                    mCargo.setIntakeRollers(CARGO_ARM.CARGOSHIP_INTAKE_OUT_ROLLER_SPEED);
                    break;
                default:
                    Logger.logError("Unexpected Cargo Arm State");
                    break;
            }
        } else {
            mCargo.setIntakeRollers(0.0);
        }

        // If in manual override mode, use open loop control for Arm (except when both
        // arms are in manual mode)
        // The normal hatch buttons now only serve to actuate the pneumatic arm
        if (mHatch.getHatchIntakeControlState() == HatchIntakeControlState.OPEN_LOOP) {
            if (mCargo.getArmControlState() != CargoArmControlState.OPEN_LOOP) {
                double movement = -operatorJoystick.getRawAxis(1) / 1.5;
                mHatch.setOpenLoop(MkMath.handleDeadband(Math.pow(movement, 3), INPUT.kOperatorDeadband));
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
            mHatch.setHatchMechanismState(mHatch.getHatchMechanismState() == HatchMechanismState.STOWED ? HatchMechanismState.GROUND_INTAKE : HatchMechanismState.STOWED);
        } else if (mSpearTogglePlaceStow.isPressed()) {
            if (mHatch.getHatchSpearState() == HatchSpearState.STOW) {
                mHatch.setHatchMechanismState(HatchMechanismState.SPEAR_PLACE_ONLY);
            } else {
                mHatch.setHatchMechanismState(HatchMechanismState.SPEAR_STOW_ONLY);
            }
        } else if (mSpearIntake.isPressed()) {
            mHatch.setHatchMechanismState(HatchMechanismState.STATION_INTAKE);
        } else if (mStowAllButton.isPressed()) {
            mHatch.setHatchMechanismState(HatchMechanismState.STOWED);
            mCargo.setArmState(CargoArmState.REVERSE_CARGOSHIP);
        }
    }
}
