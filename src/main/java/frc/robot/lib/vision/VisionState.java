package frc.robot.lib.vision;

import frc.robot.lib.util.DriveSignal;

public class VisionState {

  public static VisionState EMPTY = new VisionState(DriveSignal.BRAKE, LimelightTarget.EMPTY, 0.0);
  private DriveSignal mDriveSignal;
  private LimelightTarget mTarget;
  private double mYaw;

  public VisionState(DriveSignal mDriveSignal, LimelightTarget mTarget, double mYaw) {
    this.mDriveSignal = mDriveSignal;
    this.mTarget = mTarget;
    this.mYaw = mYaw;
  }

  public DriveSignal getDriveSignal() {
    return mDriveSignal;
  }

  public LimelightTarget getTarget() {
    return mTarget;
  }

  public double getYaw() {
    return mYaw;
  }
}
