package frc.robot.lib.math;

import frc.robot.Constants;

public class MkMath {
  public static double nativeUnitsPer100MstoInchesPerSec(double vel) {
    return 10 * nativeUnitsToInches(vel);
  }

  public static double nativeUnitsToInches(double units) {
    return (units / Constants.GENERAL.kTicksPerRev) * (Constants.DRIVE.kCircumference);
  }

  public static double InchesPerSecToUnitsPer100Ms(double vel) {
    return InchesToNativeUnits(vel) / 10;
  }

  public static double InchesToNativeUnits(double in) {
    return (Constants.GENERAL.kTicksPerRev) * (in / Constants.DRIVE.kCircumference);
  }

  public static double handleDeadband(double val, double deadband) {
    return (java.lang.Math.abs(val) > java.lang.Math.abs(deadband)) ? val : 0.0;
  }

  public static double RPMToInchesPerSec(double vel) {
    return (vel / 60.0) * Constants.DRIVE.kCircumference;
  }

  public static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60;
  }

  public static double rotationsToInches(double rotations) {
    return rotations * (Constants.DRIVE.kWheelDiameter * Math.PI);
  }

  public static double inchesPerSecondToRpm(double inches_per_second) {
    return inchesToRotations(inches_per_second) * 60;
  }

  public static double inchesToRotations(double inches) {
    return inches / (Constants.DRIVE.kWheelDiameter * Math.PI);
  }

  public static double radiansPerSecondToTicksPer100ms(double rad_s) {
    return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
  }

  public static double sin(double deg) {
    return java.lang.Math.sin(java.lang.Math.toRadians(deg));
  }

  public static double degreesToNativeUnits(double ang) {
    return (ang / 360.0) * 4096.0;
  }

  public static double nativeUnitsPer100MstoDegreesPerSec(double vel) {
    return nativeUnitsToDegrees(vel) * 10;
  }

  public static double nativeUnitsToDegrees(double raw) {
    return ((raw / 4096.0) * 360.0);
  }

  public static double degreesToPigeonNativeUnits(double angle) {
    return (angle / 360.0) * 8192.0;
  }

  public static double pigeonNativeUnitsToDegrees(double units) {
    return (units / 8192.0) * 360.0;
  }

  public static double inchesToFt(double inches) {
    return inches / 12.0;
  }

  public static double normalAbsoluteAngleDegrees(double angle) {
    return (angle %= 360) >= 0 ? angle : (angle + 360);
  }
}
