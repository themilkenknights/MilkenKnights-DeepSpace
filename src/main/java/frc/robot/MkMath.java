package frc.robot;

public class MkMath {

	public static double nativeUnitsPer100MstoInchesPerSec(double vel) {
		return 10 * nativeUnitsToInches(vel);
	}

	public static double nativeUnitsToInches(double units) {
		return (units / Constants.CODES_PER_REV) * (Constants.CIRCUMFERENCE);
	}

	public static double InchesPerSecToUnitsPer100Ms(double vel) {
		return InchesToNativeUnits(vel) / 10;
	}

	public static double InchesToNativeUnits(double in) {
		return (Constants.CODES_PER_REV) * (in / Constants.CIRCUMFERENCE);
	}

	public static double AngleToVel(double angle) {
		return (angle / 360) * Constants.TURN_IN_PLACE_CIRCUMFERENCE;
	}

	public static double handleDeadband(double val, double deadband) {
		return (java.lang.Math.abs(val) > java.lang.Math.abs(deadband)) ? val : 0.0;
	}

	public static double RPMToInchesPerSec(double vel) {
		return (vel / 60.0) * Constants.CIRCUMFERENCE;
	}

	public static double rotationsToInches(double rotations) {
		return rotations * (Constants.WHEEL_DIAMETER * Math.PI);
	}

	public static double inchesToRotations(double inches) {
		return inches / (Constants.WHEEL_DIAMETER * Math.PI);
	}

	public static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}

	public static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60;
	}

	public static double radiansPerSecondToTicksPer100ms(double rad_s) {
		return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
	}

	public static double sin(double deg) {
		return java.lang.Math.sin(java.lang.Math.toRadians(deg));
	}

}
