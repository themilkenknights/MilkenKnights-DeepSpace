package frc.robot.auto.actions;

import frc.robot.Robot;
import frc.robot.lib.util.MkTimer;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;

public class VisionDrive implements Action {
  private MkTimer expirationTimer = new MkTimer();
  private VisionGoal mGoal;

  public VisionDrive(VisionGoal mGoal) {
    this.mGoal = mGoal;
  }

  @Override
  public boolean isFinished() {
    return expirationTimer.isDone() || ((mGoal == VisionGoal.INTAKE_HATCH
        || mGoal == VisionGoal.PLACE_HATCH) && (HatchArm.getInstance().isHatchTriggeredTimer())) || (mGoal == VisionGoal.PLACE_CARGO
        && Drive.getInstance().isDriveStateFinished());
  }

  @Override
  public void update() {

  }

  @Override
  public void done() {
    if (mGoal == VisionGoal.INTAKE_HATCH) {
      HatchArm.getInstance().setHatchState(HatchArm.HatchState.STOW);
    } else if (mGoal == VisionGoal.PLACE_CARGO) {
      CargoArm.getInstance().setArmState(CargoArm.CargoArmState.REVERSE_CARGOSHIP);
    }
  }

  @Override
  public void start() {
    Drive.getInstance().cancelPath();
    expirationTimer.start(5.0);
    Drive.getInstance().setVisionDrive(mGoal);
  }

  public enum VisionGoal {
    PLACE_HATCH, INTAKE_HATCH, PLACE_CARGO
  }
}
