package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTimer;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchSpearState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.ClimbState;
import frc.robot.subsystems.Superstructure.RobotState;
import frc.robot.subsystems.Vision;

/**
 * View READEME.md to view the simple HID map
 */
public class Input {

  private static final MkJoystick mDriverJoystick = new MkJoystick(0);
  private static final MkJoystick mOperatorJoystick = new MkJoystick(1);

  private static final MkJoystickButton mToggleVelocity = mDriverJoystick.getButton(10, "Toggle Velocity Setpoint");

  private static final MkJoystickButton mHatchVisionPlace = mDriverJoystick.getButton(1, "Hatch Vision Place");

  private static final MkJoystickButton mCargoVisionOuttake = mDriverJoystick.getButton(2, "Vision Cargo Outtake");

  private static final MkJoystickButton mHatchVisionIntake = mDriverJoystick.getButton(3, "Vision Hatch Intake");

  private static final MkJoystickButton mAutoClimb = mDriverJoystick.getButton(4, "Automated Climb");

  private static final MkJoystickButton mFrontClimb = mDriverJoystick.getButton(5, "Climb Front");

  private static final MkJoystickButton mRearClimb = mDriverJoystick.getButton(6, "Climb Rear");

  private static final MkJoystickButton mCargoArmManual = mOperatorJoystick.getButton(2, "Cargo Arm Manual Mode");

  private static final MkJoystickButton mSpearTogglePlaceStow = mOperatorJoystick
      .getButton(3, "Hatch Spear Toggle (Place/Stow)");

  private static final MkJoystickButton mSpearIntake = mOperatorJoystick.getButton(4, "Hatch Spear HP Intake");

  private static final MkJoystickButton mIntakeRollerIn = mOperatorJoystick.getButton(5, "Intake Roller In");

  private static final MkJoystickButton mIntakeRollerOut = mOperatorJoystick.getButton(6, "Intake Roller Out Fast");

  private static final MkJoystickButton toggleVision = mOperatorJoystick.getButton(7, "Toggle Vision");

  private static final MkJoystickButton mZeroArmToggleLimit = mOperatorJoystick
      .getButton(8, "Zero Arm Encoders && Disable Soft Limit");

  private static final MkJoystickButton mStopAuto = mOperatorJoystick.getButton(9, "Stop Auto");

  private static final MkJoystickButton mStowAllButton = mOperatorJoystick.getButton(10, "Defense Mode - Stow All");

  private static final MkJoystickButton mGroundIntakeToggleButton = mOperatorJoystick
      .getButton(11, "Ground Intake Toggle (Stow/Ground)");

  private static final MkJoystickButton mTransferButton = mOperatorJoystick.getButton(12, "Transfer Hatch Button");

  private static MkTimer rumbleTimer = new MkTimer();

  private static boolean isVelocitySetpoint = false;

  private static Drive mDrive = Drive.getInstance();
  private static HatchArm mHatch = HatchArm.getInstance();
  private static CargoArm mCargo = CargoArm.getInstance();
  private static Superstructure mStructure = Superstructure.getInstance();
  private static Vision mVision = Vision.getInstance();

  public static void updateControlInput() {
    RobotState currentRobotState = mStructure.getRobotState();

    // Joystick isn't connected if throttle is equal to zero. Used to ensure robot
    // doesn't move when
    // Joystick unplugged.
    boolean isOperatorJoystickConnected = mOperatorJoystick.getRawAxis(3) != -1.0;

    // Stop rumble after 250ms
    if (rumbleTimer.isDone()) {
      mDriverJoystick.setRumble(RumbleType.kLeftRumble, 0.0);
      mDriverJoystick.setRumble(RumbleType.kRightRumble, 0.0);
      rumbleTimer.reset();
    }

    // Disable Auto Mode and set robot state to teleop drive for manual control
    if (mStopAuto.isPressed()) {
      mStructure.setRobotState(RobotState.TELEOP_DRIVE);
    }

    // Re-seed absolute values to relative encoder and disable soft limit for arms
    if (mZeroArmToggleLimit.isPressed()) {
      mCargo.zeroEncoder();
      mCargo.disableSoftLimit();
    }

    // Toggle Limelight LEDs
    if (toggleVision.isPressed()) {
      mVision.toggleVision();
    }

    // Move arms in open loop while held. This switches the arm to open loop control
    // mode.
    if (mCargoArmManual.isHeld()) {
      mCargo.setOpenLoop(MkMath.handleDeadband(-mOperatorJoystick.getRawAxis(1), GENERAL.kOperatorDeadband));
    }

    // Ensure that arm stops after manual mode button is released and is not set at
    // the last output
    if (mOperatorJoystick.getRawButtonReleased(2)) {
      mCargo.setOpenLoop(0.0);
    }

    if (mToggleVelocity.isPressed()) {
      isVelocitySetpoint = !isVelocitySetpoint;
    }

    // Update robot state as it might have changed
    currentRobotState = mStructure.getRobotState();

    // Enable auto climb that uses encoders & gyro
    if (mAutoClimb.isPressed()) {
      mStructure.setRobotState(RobotState.AUTO_CLIMB);
    } else if (mFrontClimb.isPressed()) {
      // Toggle Front Climb Actuators (Toward Spear)
      mStructure.setFrontClimbState(
          mStructure.getFrontClimbState() == ClimbState.RETRACTED ? ClimbState.LOWERED : ClimbState.RETRACTED);
    } else if (mRearClimb.isPressed()) {
      // Toggle Rear Climb Actuators (Toward Cargo Arm/Rio)
      mStructure.setRearClimbState(
          mStructure.getRearClimbState() == ClimbState.RETRACTED ? ClimbState.LOWERED : ClimbState.RETRACTED);
    }

    // GTA Style driving.
    // Right and Left Triggers are added (with left being negative) to get a
    // throttle value
    if (currentRobotState == RobotState.TELEOP_DRIVE) {
      double forward = (-mDriverJoystick.getRawAxis(2) + mDriverJoystick.getRawAxis(3));
      double turn = (-mDriverJoystick.getRawAxis(0));
      DriveSignal controlSig = DriveHelper.cheesyDrive(forward, turn, true);
      /*if(isVelocitySetpoint){
        if(mDriverJoystick.getPOV() == 270){
          mDrive.setDistanceAndAngle(0, -mDriverJoystick.getRawAxis(0) * 20);
        } else if(mDriverJoystick.getPOV() == 90){
          mDrive.setDistanceAndAngle(0, -mDriverJoystick.getRawAxis(0) * -20);
        }
       // mDrive.setDistanceAndAngle(forward, -mDriverJoystick.getRawAxis(0) * 25);
      } else{ */
      mDrive.setOpenLoop(controlSig);
      // }
    }
    if (isOperatorJoystickConnected) {
      if (mCargoVisionOuttake.isPressed()) {
        mStructure.setRobotState(RobotState.VISION_CARGO_OUTTAKE);
      } else if (mOperatorJoystick.getPOV() == 0) {
        mCargo.setArmState(CargoArmState.FORWARD_ROCKET_LEVEL_TWO);
      } else if (mOperatorJoystick.getPOV() == 90) {
        mCargo.setArmState(CargoArmState.REVERSE_CARGOSHIP);
      } else if (mOperatorJoystick.getPOV() == 180) {
        mCargo.setArmState(CargoArmState.FORWARD_ROCKET_LEVEL_ONE);
      } else if (mOperatorJoystick.getPOV() == 270) {
        mCargo.setArmState(CargoArmState.INTAKE);
      }

      if (mIntakeRollerIn.isHeld()) {
        mCargo.setIntakeRollers(CARGO_ARM.kIntakeRollerInSpeed);
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
      } else {
        mCargo.setIntakeRollers(0.0);
      }

      if (mHatchVisionIntake.isPressed()) {
        mStructure.setRobotState(RobotState.HATCH_VISION_INTAKE);
      } else if (mHatchVisionPlace.isPressed()) {
        mStructure.setRobotState(RobotState.HATCH_VISION_OUTTAKE);
      } else if (mSpearTogglePlaceStow.isPressed()) {
        if (mHatch.getHatchSpearState() == HatchSpearState.STOW) {
          mHatch.setHatchState(HatchSpearState.PLACE);
        } else {
          mHatch.setHatchState(HatchSpearState.STOW);
        }
      } else if (mSpearIntake.isPressed()) {
        mHatch.setHatchState(HatchSpearState.INTAKE);
      } else if (mStowAllButton.isPressed()) {
        mHatch.setHatchState(HatchSpearState.STOW);
        mCargo.setArmState(CargoArmState.REVERSE_CARGOSHIP);
      }
    }
  }

  /**
   * Rumble Xbox Controller for 250ms at 25% output
   */
  public static void rumbleDriverController() {
    mDriverJoystick.setRumble(RumbleType.kLeftRumble, 0.25);
    mDriverJoystick.setRumble(RumbleType.kRightRumble, 0.25);
    rumbleTimer.start(0.5);
  }
}
