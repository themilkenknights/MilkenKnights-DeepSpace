package frc.robot.lib.math;

import frc.robot.Constants;
import frc.robot.lib.util.DriveSignal;

public class DriveHelper {
		private static final double kThrottleDeadband = Constants.kThrottleDeadband;
		private static final double kWheelDeadband = Constants.kWheelDeadband;

		public static DriveSignal cheesyDrive(double throttle, double wheel, boolean cubeInputs) {
				double leftMotorSpeed;
				double rightMotorSpeed;
				double throttleInput = limit(throttle);
				double turnInput = limit(wheel);
				/*moveValue = handleDeadband(moveValue, kThrottleDeadband);
				rotateValue = handleDeadband(rotateValue, kWheelDeadband);
				if (cubeInputs) {
						rotateValue = rotateValue * rotateValue * rotateValue;
				}
				rotateValue = rotateValue / 1.5;
				if (moveValue > 0.0) {
						if (rotateValue > 0.0) {
								leftMotorSpeed = moveValue - rotateValue;
								rightMotorSpeed = Math.max(moveValue, rotateValue);
						} else {
								leftMotorSpeed = Math.max(moveValue, -rotateValue);
								rightMotorSpeed = moveValue + rotateValue;
						}
				} else {
						if (rotateValue > 0.0) {
								leftMotorSpeed = -Math.max(-moveValue, rotateValue);
								rightMotorSpeed = moveValue + rotateValue;
						} else {
								leftMotorSpeed = moveValue - rotateValue;
								rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
						}
				} */



				double saturatedInput;
				double greaterInput = Math.max(Math.abs(throttleInput), Math.abs(turnInput));
				//range [0, 1]
				double lesserInput = Math.abs(throttleInput) + Math.abs(turnInput) - greaterInput;
				//range [0, 1]
				if (greaterInput > 0.0) {
						saturatedInput = (lesserInput / greaterInput) + 1.0;
						//range [1, 2]
				} else {
						saturatedInput = 1.0;
				}

				throttleInput = throttleInput / saturatedInput;
				turnInput = turnInput / saturatedInput;


				leftMotorSpeed = throttleInput + turnInput;
				rightMotorSpeed = throttleInput - turnInput;
				
				return new DriveSignal(leftMotorSpeed, rightMotorSpeed);
		}

		protected static double limit(double num) {
				if (num > 1.0) {
						return 1.0;
				}
				if (num < -1.0) {
						return -1.0;
				}
				return num;
		}

		public static double handleDeadband(double val, double deadband) {
				return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
		}
}
