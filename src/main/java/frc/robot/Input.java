package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.drivers.MkJoystick;
import frc.robot.lib.drivers.MkJoystickButton;
import frc.robot.lib.math.DriveHelper;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.LatchedBoolean;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTimer;
import frc.robot.lib.util.SynchronousPIDF;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.HatchArm.HatchState;
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

  //Driver Controls
  private static final MkJoystickButton mToggleManualVision = mDriverJoystick.getButton(5, "Toggle Vision On");
  private static final MkJoystickButton mToggleManualVisionOff = mDriverJoystick.getButton(6, "Toggle Vision Off");

  private static final MkJoystickButton mHatchVisionPlace = mDriverJoystick.getButton(1, "Hatch Vision Place");
  private static final MkJoystickButton mCargoVisionOuttake = mDriverJoystick.getButton(3, "Vision Cargo Outtake");
  private static final MkJoystickButton mHatchVisionIntake = mDriverJoystick.getButton(2, "Vision Hatch Intake");
  private static final MkJoystickButton mAutoClimb = mDriverJoystick.getButton(4, "Automated Climb");
  private static final MkJoystickButton mFrontClimb = mOperatorJoystick.getButton(7, "Climb Front");
  private static final MkJoystickButton mRearClimb = mOperatorJoystick.getButton(8, "Climb Rear");

  //Operator Controls
  private static final MkJoystickButton mCargoArmManual = mOperatorJoystick.getButton(2, "Cargo Arm Manual Mode");
  private static final MkJoystickButton mSpearTogglePlaceStow = mOperatorJoystick.getButton(3, "Hatch Spear Toggle (Place/Stow)");
  private static final MkJoystickButton mSpearIntake = mOperatorJoystick.getButton(4, "Hatch Spear HP Intake");
  private static final MkJoystickButton mIntakeRollerIn = mOperatorJoystick.getButton(5, "Intake Roller In");
  private static final MkJoystickButton mIntakeRollerOut = mOperatorJoystick.getButton(6, "Intake Roller Out Fast");
  private static final MkJoystickButton frontCargoshipOuttake = mOperatorJoystick.getButton(11, "Front Cargoship Outtake");
  private static final MkJoystickButton mZeroArmToggleLimit = mOperatorJoystick.getButton(10, "Zero Arm Encoders && Disable Soft Limit");
  private static final MkJoystickButton mStowAllButton = mOperatorJoystick.getButton(9, "Defense Mode - Stow All");
  private static final MkJoystickButton mRetractPancake = mOperatorJoystick.getButton(12, "Retract Pancake Actuator");

  private static final LatchedBoolean cancelAutoTrigger = new LatchedBoolean();
  private static boolean isManualVisionMode;
  private static SynchronousPIDF mVisionAssist = new SynchronousPIDF(0.0157, 0.0, 275.0);
  private static MkTimer rumbleTimer = new MkTimer();
  private static Drive mDrive = Drive.getInstance();
  private static HatchArm mHatch = HatchArm.getInstance();
  private static CargoArm mCargo = CargoArm.getInstance();
  private static Superstructure mStructure = Superstructure.getInstance();
  private static Vision mVision = Vision.getInstance();

  public static void updateControlInput() {

    // Joystick isn't connected if throttle is equal to zero. Used to ensure robot doesn't move when Joystick unplugged.
    boolean isOperatorJoystickConnected = mOperatorJoystick.getRawAxis(3) != 0.0;
    // Stop rumble after timer is finished
    if (rumbleTimer.isDone()) {
      mDriverJoystick.setRumble(RumbleType.kLeftRumble, 0.0);
      mDriverJoystick.setRumble(RumbleType.kRightRumble, 0.0);
      rumbleTimer.reset();
    }

    // Disable Auto Mode and set robot state to teleop drive for manual control
    if (cancelAutoTrigger.update(mOperatorJoystick.getTriggerPressed())) {
      mStructure.setRobotState(RobotState.TELEOP_DRIVE);
      mHatch.retractPancakeActuator();
    }

    // Re-seed absolute values to relative encoder and disable soft limit for arms
    if (mZeroArmToggleLimit.isPressed()) {
      mCargo.zeroEncoder();
      mCargo.disableSoftLimit();
    }

    if (mToggleManualVision.isPressed()) {
      isManualVisionMode = true;
      mVisionAssist.reset();
    } else if (mToggleManualVisionOff.isPressed()) {
      isManualVisionMode = false;
      mVisionAssist.reset();
    }

    // Toggle Limelight LEDs
   /* if (toggleVision.isPressed()) {
      mVision.toggleVision();
    } */

    // Move arms in open loop while held. This switches the arm to open loop control mode.
    if (mCargoArmManual.isHeld()) {
      mCargo.setOpenLoop(MkMath.handleDeadband(-mOperatorJoystick.getRawAxis(1), GENERAL.kOperatorDeadband));
    }

    // Ensure that arm stops after manual mode button is released and is not set at the last output
    if (mOperatorJoystick.getRawButtonReleased(2)) {
      mCargo.setOpenLoop(0.0);
    }

    // Enable auto climb that uses encoders & gyro
    if (mAutoClimb.isPressed()) {
      mStructure.setRobotState(RobotState.AUTO_CLIMB);
    } else if (mFrontClimb.isPressed()) {
      // Toggle Front Climb Actuators (Toward Hatch Spear)
      mStructure.setFrontClimbState(mStructure.getFrontClimbState() == ClimbState.RETRACTED ? ClimbState.LOWERED : ClimbState.RETRACTED);
    } else if (mRearClimb.isPressed()) {
      // Toggle Rear Climb Actuators (Toward Cargo Arm/Rio)
      mStructure.setRearClimbState(mStructure.getRearClimbState() == ClimbState.RETRACTED ? ClimbState.LOWERED : ClimbState.RETRACTED);
    }

    RobotState currentRobotState = mStructure.getRobotState();

    // GTA Style driving: Right and Left Triggers are added (with left being negative) to get a throttle value
    if (currentRobotState == RobotState.TELEOP_DRIVE) {
      SmartDashboard.putBoolean("Vision Assist", isManualVisionMode);
      setTeleopDrive();
      if (isOperatorJoystickConnected) {
        if (mCargoVisionOuttake.isPressed()) {

          // Auto align and drive towards cargoship
          mStructure.setRobotState(RobotState.VISION_CARGO_OUTTAKE);
        } else if (mOperatorJoystick.getPOV() == 0) {

          // Move cargo arm based on POV 'hat'
          mCargo.setArmState(CargoArmState.FORWARD_ROCKET_LEVEL_TWO);
        } else if (mOperatorJoystick.getPOV() == 90) {
          mCargo.setArmState(CargoArmState.REVERSE_CARGOSHIP);
        } else if (mOperatorJoystick.getPOV() == 180) {
          mCargo.setArmState(CargoArmState.FORWARD_ROCKET_LEVEL_ONE);
        } else if (mOperatorJoystick.getPOV() == 270) {
          mCargo.setArmState(CargoArmState.INTAKE);
        }
        if (mIntakeRollerIn.isHeld()) {

          // Always intake at the same speed
          mCargo.setIntakeRollers(CARGO_ARM.kIntakeRollerInSpeed);
        } else if (mIntakeRollerOut.isHeld()) {

          // Set outtake roller speed based on Cargo Arm Position
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
        } else if (frontCargoshipOuttake.isHeld()) {
          mCargo.setIntakeRollers(CARGO_ARM.kFrontCargoShipIntakeRollerOut);
        } else {
          mCargo.setIntakeRollers(0.0);
        }
        if (mHatchVisionIntake.isPressed()) {
          mStructure.setRobotState(RobotState.HATCH_VISION_INTAKE);
        } else if (mHatchVisionPlace.isPressed()) {
          mStructure.setRobotState(RobotState.HATCH_VISION_OUTTAKE);
        } else if (mSpearTogglePlaceStow.isPressed()) {

          // Toggle between stow and place
          if (mHatch.getHatchSpearState() == HatchState.STOW) {

            // Place mode uses the limit switch to retract the pancake actuator at the correct time
            mHatch.setHatchState(HatchState.PLACE);
          } else {

            // Stows hatch spear and extends pancake actuator
            mHatch.setHatchState(HatchState.STOW);
          }
        } else if (mSpearIntake.isPressed()) {

          // Intake mode uses the limit switch to stow arm at the correct time
          mHatch.setHatchState(HatchState.INTAKE);
        } else if (mStowAllButton.isPressed()) {

          mHatch.setHatchState(HatchState.STOW);
          mCargo.setArmState(CargoArmState.REVERSE_CARGOSHIP);
        } else if (mRetractPancake.isPressed()) {
          mHatch.retractPancakeActuator();
        }
      }
    } else if (mDrive.getDriveControlState() == Drive.DriveControlState.VISION_DRIVE) {

      //Weird hack as running from non-static method appears to cause issues with the PID controller
      mDrive.updateVisionDrive();
    }
  }

  /**
   * Rumble Xbox Controller for 250ms at 25% output
   */
  public static void rumbleDriverController(double intensity, double sec) {
    mDriverJoystick.setRumble(RumbleType.kLeftRumble, intensity);
    mDriverJoystick.setRumble(RumbleType.kRightRumble, intensity);
    rumbleTimer.start(sec);
  }

  private static void setTeleopDrive() {
    double forward = (-mDriverJoystick.getRawAxis(2) + mDriverJoystick.getRawAxis(3));
    double turn = (-mDriverJoystick.getRawAxis(0));
    DriveSignal controlSig = DriveHelper.cheesyDrive(forward, turn, true);
    if (isManualVisionMode) {
      double visionTurn = 0.0;
      LimelightTarget target = mVision.getLimelightTarget();
      if (target.isValidTarget()) {
        if (mHatch.getHatchSpearState() != HatchState.PLACE && target.getDistance() < 80.0) {
          visionTurn = mVisionAssist.calculate(Vision.getInstance().getLimelightTarget().getYaw());
        }
      }
      /*if (HatchArm.getInstance().isHatchTriggeredTimer(0.3)) {
        isManualVisionMode = false;
        mDrive.setOpenLoop(DriveSignal.BRAKE);
      } else {*/
      mDrive.setOpenLoop(new DriveSignal(controlSig.getLeft() - visionTurn, controlSig.getRight() + visionTurn));
      // }
    } else {
      mDrive.setOpenLoop(controlSig);
    }
  }
}
