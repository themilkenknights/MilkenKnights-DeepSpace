package frc.robot.lib.util;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
public class DriveSignal {

  public static DriveSignal BRAKE = new DriveSignal(0, 0, NeutralMode.Brake);
  protected double mLeftMotor;
  protected double mRightMotor;
  protected NeutralMode mBrakeMode;
  protected double mAngle;

  public DriveSignal(double left, double right) {
    this(left, right, NeutralMode.Brake);
  }

  public DriveSignal(double left, double right, NeutralMode brakeMode) {
    mLeftMotor = left;
    mRightMotor = right;
    mBrakeMode = brakeMode;
  }

  public DriveSignal(double left, double right, double angle) {
    mLeftMotor = left;
    mRightMotor = right;
    mAngle = angle;
    mBrakeMode = NeutralMode.Brake;
  }

  public double getLeft() {
    return mLeftMotor;
  }

  public double getRight() {
    return mRightMotor;
  }

  public NeutralMode getBrakeMode() {
    return mBrakeMode;
  }

  @Override
  public String toString() {
    return "L: " + mLeftMotor + ", R: " + mRightMotor + ", BRAKE: " + mBrakeMode.toString();
  }
}
