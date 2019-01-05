package frc.robot.util.math;

import frc.robot.Constants.INPUT;
import frc.robot.util.state.DriveSignal;

public class DriveHelper {

    private static final double kThrottleDeadband = INPUT.kThrottleDeadband;
    private static final double kWheelDeadband = INPUT.kWheelDeadband;

    public static DriveSignal cheesyDrive(double throttle, double wheel, boolean cubeInputs) {

        double leftMotorSpeed;
        double rightMotorSpeed;

        double moveValue = limit(throttle);
        double rotateValue = limit(wheel);

        moveValue = handleDeadband(moveValue, kThrottleDeadband);
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
        }

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
