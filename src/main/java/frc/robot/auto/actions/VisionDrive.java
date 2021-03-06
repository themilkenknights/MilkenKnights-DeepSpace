package frc.robot.auto.actions;

import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;

public class VisionDrive implements Action {
  private VisionGoal mGoal;
  private double maxTime;

  public VisionDrive(VisionGoal mGoal) {
    this(mGoal, 2.0);
  }

  public VisionDrive(VisionGoal mGoal, double maxTime) {
    this.mGoal = mGoal;
    this.maxTime = maxTime;
  }

  @Override
  public boolean isFinished() {
    return Drive.getInstance().isDriveStateFinished();
  }

  @Override
  public void update() {

  }

  @Override
  public void done() {
    if (mGoal == VisionGoal.INTAKE_HATCH) {
      Logger.logMarker("End Vision Drive Intake, Stowing Hatch");
      HatchArm.getInstance().setHatchState(HatchArm.HatchState.STOW);
    } else if (mGoal == VisionGoal.PLACE_CARGO) {
      CargoArm.getInstance().setArmState(CargoArm.CargoArmState.REVERSE_CARGOSHIP);
      Drive.getInstance().setOpenLoop(DriveSignal.BRAKE);
    }
  }

  @Override
  public void start() {
    Drive.getInstance().cancelPath();
    Logger.logMarker("Starting Vision Drive");
    Drive.getInstance().setVisionDrive(mGoal, maxTime);
  }

  public enum VisionGoal {
    PLACE_HATCH, INTAKE_HATCH, PLACE_CARGO
  }
}
