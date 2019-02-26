package frc.robot;

import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.INPUT;
import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchMechanismState;
import frc.robot.subsystems.HatchArm.HatchSpearState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.Vision;

/**
 * View READEME.md to view the simple HID map
 */
public class Input {

    private static final MkJoystick driverJoystick = new MkJoystick(0);
    private static final MkJoystick operatorJoystick = new MkJoystick(1);

    private static final MkJoystickButton mAutoClimb = driverJoystick.getButton(4, "Automated Climb");

    private static final MkJoystickButton mHatchVisionPlace = driverJoystick.getButton(1, "Hatch Vision Place");

    private static final MkJoystickButton mCargoVisionOuttake = driverJoystick.getButton(2, "Vision Cargo Outtake");

    private static final MkJoystickButton mHatchVisionIntake = driverJoystick.getButton(3, "Climb Front");


    //Trigger is Kettering Manual while held

    private static final MkJoystickButton mCargoArmManual = operatorJoystick.getButton(2, "Cargo Arm Manual Mode");

    private static final MkJoystickButton mSpearTogglePlaceStow = operatorJoystick.getButton(3, "Hatch Spear Toggle (Place/Stow)");

    private static final MkJoystickButton mSpearIntake = operatorJoystick.getButton(4, "Hatch Spear HP Intake");

    private static final MkJoystickButton mIntakeRollerIn = operatorJoystick.getButton(5, "Intake Roller In");

    private static final MkJoystickButton mIntakeRollerOut = operatorJoystick.getButton(6, "Intake Roller Out Fast");

    private static final MkJoystickButton mZeroArms = operatorJoystick.getButton(8, "Zero Arm Encoders");

    private static final MkJoystickButton mStopAuto = operatorJoystick.getButton(10, "Stop Auto");

    private static final MkJoystickButton mStowAllButton = operatorJoystick.getButton(10, "Defense Mode - Stow All");

    private static final MkJoystickButton mGroundIntakeToggleButton = operatorJoystick.getButton(11, "Ground Intake Toggle (Stow/Ground)");

    private static final MkJoystickButton mTransferButton = operatorJoystick.getButton(12, "Transfer Hatch Button");

    private static Drive mDrive = Drive.getInstance();
    private static HatchArm mHatch = HatchArm.getInstance();
    private static CargoArm mCargo = CargoArm.getInstance();
    private static Superstructure mStructure = Superstructure.getInstance();
    private static Vision mVision = Vision.getInstance();

    public static void updateControlInput() {
        RobotState currentRobotState = mStructure.getRobotState();
        boolean isVisionState = currentRobotState == RobotState.VISION_CARGO_INTAKE || currentRobotState == RobotState.VISION_CARGO_OUTTAKE
            || currentRobotState == RobotState.HATCH_VISION_INTAKE || currentRobotState == RobotState.HATCH_VISION_OUTTAKE;

        if (isVisionState && mStopAuto.isPressed()) {
            mStructure.setRobotState(RobotState.TELEOP_DRIVE);
        }

        if (mZeroArms.isPressed()) {
            mCargo.zeroEncoder();
            mHatch.zeroEncoder();
        }

        if (mCargoArmManual.isHeld()) {
            mCargo.setOpenLoop(MkMath.handleDeadband(-operatorJoystick.getRawAxis(1), INPUT.kOperatorDeadband));
            mVision.configCargoStream();
        } else if (operatorJoystick.getTrigger()) {
            double movement = -operatorJoystick.getRawAxis(1) / 1.5;
            mHatch.setOpenLoop(MkMath.handleDeadband(Math.pow(movement, 3), INPUT.kOperatorDeadband));
            mVision.configHatchStream();
        }

        if (operatorJoystick.getRawButtonReleased(2)) {
            mCargo.setOpenLoop(0.0);
        }

        if(operatorJoystick.getTriggerReleased()){
            mHatch.setOpenLoop(0.0);
        }

        currentRobotState = mStructure.getRobotState();

        if (currentRobotState == RobotState.TELEOP_DRIVE) {
            double forward = (-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
            double turn = (-driverJoystick.getRawAxis(0));
            DriveSignal controlSig = DriveHelper.cheesyDrive(forward, turn, true);
            mDrive.setOpenLoop(controlSig);
        }

        if (mCargoVisionOuttake.isPressed()) {
            mStructure.setRobotState(RobotState.VISION_CARGO_OUTTAKE);
            mVision.configCargoStream();
        } else if (operatorJoystick.getPOV() == 0) {
            mCargo.setArmState(CargoArmState.FORWARD_ROCKET_LEVEL_TWO);
            mVision.configCargoStream();
        } else if (operatorJoystick.getPOV() == 90) {
            mCargo.setArmState(CargoArmState.REVERSE_CARGOSHIP);
            mVision.configCargoStream();
        } else if (operatorJoystick.getPOV() == 180) {
            mCargo.setArmState(CargoArmState.FORWARD_ROCKET_LEVEL_ONE);
            mVision.configCargoStream();
        } else if (operatorJoystick.getPOV() == 270) {
            mCargo.setArmState(CargoArmState.INTAKE);
            mVision.configCargoStream();
        }


        if (mIntakeRollerIn.isHeld()) {
            mCargo.setIntakeRollers(CARGO_ARM.kIntakeRollerInSpeed);
            mVision.configCargoStream();
        } else if (mIntakeRollerOut.isHeld()) {
            switch (mCargo.getArmState()) {
                case INTAKE:
                case ENABLE:
                    mCargo.setIntakeRollers(CARGO_ARM.kDefaultIntakeRollerOutSpeed);
                    break;
                case FORWARD_ROCKET_LEVEL_ONE:
                    mCargo.setIntakeRollers(CARGO_ARM.kRocketLevelOneOutSpeed);
                    break;
                case FORWARD_ROCKET_LEVEL_TWO:
                    mCargo.setIntakeRollers(CARGO_ARM.kRocketLevelTwoOutSpeed);
                    break;
                case REVERSE_CARGOSHIP:
                    mCargo.setIntakeRollers(CARGO_ARM.kCargoShipIntakeRollerOut);
                    break;
                default:
                    Logger.logError("Unexpected Cargo Arm State");
                    break;
            }
            mVision.configCargoStream();
        } else {
            mCargo.setIntakeRollers(0.0);
        }

        if (mHatchVisionIntake.isPressed()) {
            mStructure.setRobotState(RobotState.HATCH_VISION_INTAKE);
            mVision.configHatchStream();
        } else if (mHatchVisionPlace.isPressed()) {
            mStructure.setRobotState(RobotState.HATCH_VISION_OUTTAKE);
            mVision.configHatchStream();
        } else if (mTransferButton.isPressed()) {
            mHatch.setHatchMechanismState(HatchMechanismState.TRANSFER);
            mVision.configHatchStream();
        } else if (mGroundIntakeToggleButton.isPressed()) {
            mHatch.setHatchMechanismState(
                mHatch.getHatchMechanismState() == HatchMechanismState.GROUND_INTAKE ? HatchMechanismState.STOWED : HatchMechanismState.GROUND_INTAKE);
        } else if (mSpearTogglePlaceStow.isPressed()) {
            if (mHatch.getHatchSpearState() == HatchSpearState.STOW) {
                mHatch.setHatchMechanismState(HatchMechanismState.SPEAR_PLACE_ONLY);
            } else {
                mHatch.setHatchMechanismState(HatchMechanismState.SPEAR_STOW_ONLY);
            }
            mVision.configHatchStream();
        } else if (mSpearIntake.isPressed()) {
            mHatch.setHatchMechanismState(HatchMechanismState.STATION_INTAKE);
            mVision.configHatchStream();
        } else if (mStowAllButton.isPressed()) {
            mHatch.setHatchMechanismState(HatchMechanismState.STOWED);
            mCargo.setArmState(CargoArmState.REVERSE_CARGOSHIP);
        }
    }
}
