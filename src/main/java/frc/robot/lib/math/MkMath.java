package frc.robot.lib.math;

import frc.robot.Constants;

public class MkMath {
		public static double nativeUnitsPer100MstoInchesPerSec(double vel) {
				return 10 * nativeUnitsToInches(vel);
		}

		public static double nativeUnitsToInches(double units) {
				return (units / Constants.kTicksPerRev) * (Constants.kCircumference);
		}

		public static double InchesPerSecToUnitsPer100Ms(double vel) {
				return InchesToNativeUnits(vel) / 10;
		}

		public static double InchesToNativeUnits(double in) {
				return (Constants.kTicksPerRev) * (in / Constants.kCircumference);
		}

		public static double AngleToVel(double angle) {
				return (angle / 360) * Constants.kTurnInPlaceCircumference;
		}

		public static double handleDeadband(double val, double deadband) {
				return (java.lang.Math.abs(val) > java.lang.Math.abs(deadband)) ? val : 0.0;
		}

		public static double RPMToInchesPerSec(double vel) {
				return (vel / 60.0) * Constants.kCircumference;
		}

		public static double rpmToInchesPerSecond(double rpm) {
				return rotationsToInches(rpm) / 60;
		}

		public static double rotationsToInches(double rotations) {
				return rotations * (Constants.kWheelDiameter * Math.PI);
		}

		public static double inchesPerSecondToRpm(double inches_per_second) {
				return inchesToRotations(inches_per_second) * 60;
		}

		public static double inchesToRotations(double inches) {
				return inches / (Constants.kWheelDiameter * Math.PI);
		}

		public static double radiansPerSecondToTicksPer100ms(double rad_s) {
				return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
		}

		public static double sin(double deg) {
				return java.lang.Math.sin(java.lang.Math.toRadians(deg));
		}
}
