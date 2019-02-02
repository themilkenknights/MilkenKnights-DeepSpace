package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.HatchArm.HatchMechanismState;

public class Superstructure extends Subsystem {

  private PowerDistributionPanel mPDP;
  private Compressor mCompressor;

  private RobotState mRobotState = RobotState.TELEOP_DRIVE;

  private Superstructure() {
    mPDP = new PowerDistributionPanel(Constants.CAN.kPowerDistributionPanelID);
    mCompressor = new Compressor();
  }

  public static Superstructure getInstance() {
    return InstanceHolder.mInstance;
  }

  @Override
  public void outputTelemetry() {
    SmartDashboard.putString("Robot State", Robot.mMatchState.toString());
    SmartDashboard.putNumber("Compressor Current", mCompressor.getCompressorCurrent());
  }

  @Override
  public void zero(double timestamp) {
  }

  @Override
  public void onLoop(double timestamp) {

    switch (mRobotState) {
      case TELEOP_DRIVE:
        Drive.getInstance().mDriveControlState = DriveControlState.OPEN_LOOP;
        HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.STOWED);
        CargoArm.getInstance().setArmState(CargoArmState.STOW);
      case VISION_INTAKE_STATION:
        if (HatchArm.getInstance().hatchOnArmLimit()) {
          Drive.getInstance().setOpenLoop(DriveSignal.BRAKE);
          HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.STOWED);
        }
      default:
        Logger.logError("Unexpected robot state: " + mRobotState);
        break;
    }
  }

  @Override
  public void onStop(double timestamp) {
  }

  @Override
  public boolean checkSystem() {
    return mCompressor.getCompressorCurrent() > 0;
  }

  public void setRobotState(RobotState mRobotState) {
    switch (mRobotState) {
      case TELEOP_DRIVE:
        Drive.getInstance().mDriveControlState = DriveControlState.OPEN_LOOP;
        HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.STOWED);
        CargoArm.getInstance().setArmState(CargoArmState.STOW);
      case VISION_INTAKE_STATION:
        Drive.getInstance().startVisionTracking();
      default:
        Logger.logError("Unexpected robot state: " + mRobotState);
        break;
    }
  }

  private static class InstanceHolder {

    private static final Superstructure mInstance = new Superstructure();
  }

  public enum RobotState {
    TELEOP_DRIVE, VISION_INTAKE_STATION, VISION_PLACING, VISION_CARGO_INTAKE, VISION_CARGO_OUTTAKE
  }
}
