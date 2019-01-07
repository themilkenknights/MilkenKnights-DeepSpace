package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DRIVE;
import frc.robot.RobotState;
import frc.robot.RobotState.DriveControlState;
import frc.robot.util.drivers.MkGyro;
import frc.robot.util.drivers.MkTalon;
import frc.robot.util.drivers.MkTalon.TalonPosition;
import frc.robot.util.logging.Log;
import frc.robot.util.math.InterpolatingDouble;
import frc.robot.util.state.DriveSignal;
import frc.robot.util.structure.Subsystem;
import frc.robot.util.structure.loops.Loop;
import frc.robot.util.structure.loops.Looper;

public class Drive extends Subsystem {

    private final MkTalon leftDrive, rightDrive;
    private final MkGyro navX;
    private DriveSignal currentSetpoint;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");

    private Drive() {
        leftDrive = new MkTalon(DRIVE.LEFT_MASTER_ID, DRIVE.LEFT_SLAVE_ID, TalonPosition.Left);
        rightDrive = new MkTalon(DRIVE.RIGHT_MASTER_ID, DRIVE.RIGHT_SLAVE_ID, TalonPosition.Right);
        leftDrive.setPIDF();
        rightDrive.setPIDF();
        leftDrive.resetEncoder();
        rightDrive.resetEncoder();
        navX = new MkGyro(Port.kMXP);

        leftDrive.invertMaster(DRIVE.LEFT_MASTER_INVERT);
        leftDrive.invertSlave(DRIVE.LEFT_SLAVE_INVERT);
        leftDrive.setSensorPhase(DRIVE.LEFT_INVERT_SENSOR);

        rightDrive.invertMaster(DRIVE.RIGHT_MASTER_INVERT);
        rightDrive.invertSlave(DRIVE.RIGHT_SLAVE_INVERT);
        rightDrive.setSensorPhase(DRIVE.RIGHT_INVERT_SENSOR);
        currentSetpoint = DriveSignal.BRAKE;
    }

    public static Drive getInstance() {
        return InstanceHolder.mInstance;
    }

    /* Controls Drivetrain in PercentOutput Mode (without closed loop control) */
    public synchronized void setOpenLoop(DriveSignal signal) {
        RobotState.mDriveControlState = DriveControlState.OPEN_LOOP;
        SmartDashboard.putNumber("Left Drive Sig", signal.getLeft());
        leftDrive.set(ControlMode.PercentOutput, signal.getLeft(), signal.getBrakeMode());
        rightDrive.set(ControlMode.PercentOutput, signal.getRight(), signal.getBrakeMode());
        currentSetpoint = signal;
    }

    /**
     * Controls Drivetrain in Closed-loop velocity Mode Method sets Talons in Native
     * Units per 100ms
     *
     * @param signal An object that contains left and right velocities (inches per
     *               sec)
     */

    public synchronized void setVelocitySetpoint(DriveSignal signal, double leftFeed, double rightFeed) {
        if (RobotState.mDriveControlState == DriveControlState.PATH_FOLLOWING) {
            leftDrive.set(ControlMode.Velocity, signal.getLeftNativeVelTraj(), signal.getBrakeMode(), leftFeed);
            rightDrive.set(ControlMode.Velocity, signal.getRightNativeVelTraj(), signal.getBrakeMode(), rightFeed);
        } else {
            RobotState.mDriveControlState = DriveControlState.VELOCITY_SETPOINT;
            leftDrive.set(ControlMode.Velocity, signal.getLeftNativeVel(), signal.getBrakeMode());
            rightDrive.set(ControlMode.Velocity, signal.getRightNativeVel(), signal.getBrakeMode());
        }
        currentSetpoint = signal;
    }

    @Override
    public void outputToSmartDashboard() {
        leftDrive.updateSmartDash();
        rightDrive.updateSmartDash();
        SmartDashboard.putString("Drive State", RobotState.mDriveControlState.toString());
        SmartDashboard.putBoolean("Drivetrain Status",
                leftDrive.isEncoderConnected() && rightDrive.isEncoderConnected());
        SmartDashboard.putNumber("Current Difference", leftDrive.getCurrentOutput() - rightDrive.getCurrentOutput());
    }

    @Override
    public void slowUpdate(double timestamp) {

    }

    @Override
    public void checkSystem() {
        boolean check = true;
        leftDrive.resetEncoder();
        leftDrive.setCoastMode();
        leftDrive.setSlaveTalon(ControlMode.PercentOutput, 0);
        leftDrive.setMasterTalon(ControlMode.PercentOutput, 1);
        Timer.delay(2.0);
        if (leftDrive.getPosition() < Constants.DRIVE.MIN_TEST_POS
                || leftDrive.getSpeed() < Constants.DRIVE.MIN_TEST_VEL) {
            Log.verbose("FAILED - LEFT MASTER DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
            Log.marker("Left Master Drive Test Failed - Vel: " + leftDrive.getSpeed() + " Pos: "
                    + leftDrive.getPosition());
            check = false;
        } else {
            Log.marker(
                    "Left Master Position: " + leftDrive.getPosition() + " Left Master Speed: " + leftDrive.getSpeed());
        }

        leftDrive.setSlaveTalon(ControlMode.PercentOutput, 0);
        leftDrive.setMasterTalon(ControlMode.PercentOutput, 0);
        leftDrive.resetEncoder();
        Timer.delay(1.0);

        leftDrive.setMasterTalon(ControlMode.PercentOutput, 0);
        leftDrive.setSlaveTalon(ControlMode.PercentOutput, 1);
        Timer.delay(2.0);
        if (leftDrive.getPosition() < Constants.DRIVE.MIN_TEST_POS
                || leftDrive.getSpeed() < Constants.DRIVE.MIN_TEST_VEL) {
            Log.verbose("FAILED - LEFT SLAVE DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
            Log.marker(
                    "Left Slave Drive Test Failed - Vel: " + leftDrive.getSpeed() + " Pos: " + leftDrive.getPosition());
            check = false;
        } else {
            Log.marker(
                    "Left Slave Position: " + leftDrive.getPosition() + " Left Slave Speed: " + leftDrive.getSpeed());
        }
        leftDrive.setSlaveTalon(ControlMode.PercentOutput, 0);
        leftDrive.setMasterTalon(ControlMode.PercentOutput, 0);
        leftDrive.resetEncoder();
        Timer.delay(1.0);

        rightDrive.setCoastMode();
        rightDrive.setSlaveTalon(ControlMode.PercentOutput, 0);
        rightDrive.setMasterTalon(ControlMode.PercentOutput, 1);
        Timer.delay(2.0);
        if (rightDrive.getPosition() < Constants.DRIVE.MIN_TEST_POS
                || rightDrive.getSpeed() < Constants.DRIVE.MIN_TEST_VEL) {
            Log.verbose("FAILED - RIGHT MASTER DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
            Log.marker("Right Drive Test Failed - Vel: " + leftDrive.getSpeed() + " Pos: " + leftDrive.getPosition());
            check = false;
        } else {
            Log.marker("Right Master Position: " + rightDrive.getPosition() + " Right Master Speed: "
                    + rightDrive.getSpeed());
        }

        rightDrive.setSlaveTalon(ControlMode.PercentOutput, 0);
        rightDrive.setMasterTalon(ControlMode.PercentOutput, 0);
        rightDrive.resetEncoder();
        Timer.delay(1.0);

        rightDrive.setMasterTalon(ControlMode.PercentOutput, 0);
        rightDrive.setSlaveTalon(ControlMode.PercentOutput, 1);
        Timer.delay(2.0);
        if (rightDrive.getPosition() < Constants.DRIVE.MIN_TEST_POS
                || rightDrive.getSpeed() < Constants.DRIVE.MIN_TEST_VEL) {
            Log.verbose("FAILED - RIGHT SLAVE DRIVE FAILED TO REACH REQUIRED SPEED OR POSITION");
            Log.marker("Right Drive Test Failed - Vel: " + rightDrive.getSpeed() + " Pos: " + rightDrive.getPosition());
            check = false;
        } else {
            Log.marker("Right Slave Position: " + rightDrive.getPosition() + " Right Slave Speed: "
                    + rightDrive.getSpeed());
        }
        rightDrive.setMasterTalon(ControlMode.PercentOutput, 0);
        rightDrive.setSlaveTalon(ControlMode.PercentOutput, 0);

        if (!navX.isConnected()) {
            System.out.println("FAILED - NAVX DISCONNECTED");
            check = false;
        }

        if (check) {
            Log.verbose("Drive Test Success");
        }

        leftDrive.resetConfig();
        rightDrive.resetConfig();
    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        Loop mLoop = new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (Drive.this) {
                    leftDrive.resetEncoder();
                    rightDrive.resetEncoder();
                    navX.zeroYaw();
                }
            }

            /**
             * Updated from mEnabledLoop in Robot.java Controls drivetrain during Path
             * Following and Turn In Place and logs Drivetrain data in all modes
             * 
             * @param timestamp In Seconds Since Code Start
             */
            @Override
            public void onLoop(double timestamp) {
                synchronized (Drive.this) {
                    switch (RobotState.mDriveControlState) {
                    case OPEN_LOOP:
                        return;
                    case VELOCITY_SETPOINT:
                        return;
                    case PATH_FOLLOWING:
                        return;
                    case VISION_TRACKING:
                        updateVision();
                        return;
                    default:
                        Log.marker("Unexpected drive control state: " + RobotState.mDriveControlState);
                        break;
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                setOpenLoop(DriveSignal.BRAKE);

            }
        };
        enabledLooper.register(mLoop);
    }

    public double getYaw() {
        return navX.getYaw();
    }

    public void updateVision() {
        InterpolatingDouble result = Constants.visionDistMap.getInterpolated(new InterpolatingDouble(1.0));
        double x = tx.getDouble(0);
        double y = ty.getDouble(0);
        double steering_adjust = 0.08 * x;
        double distance_adjust = 0.0 * y;
        double left_command = -steering_adjust;
        double right_command = steering_adjust;
        setOpenLoop(new DriveSignal(left_command, right_command));

        // leftDrive.set(ControlMode.MotionMagic, 1, true, 0);
        // rightDrive.set(ControlMode.MotionMagic, 1, true, 0);
    }

    /*
     * Change Talon PID Constants to reduce oscillation during teleop driving
     */
    public void configVelocityControl() {
        leftDrive.configTeleopVelocity();
        rightDrive.configTeleopVelocity();
    }

    public boolean gyroConnected() {
        return navX.isConnected();
    }

    public boolean isEncodersConnected() {
        return leftDrive.isEncoderConnected() && rightDrive.isEncoderConnected();
    }

    private static class InstanceHolder {

        private static final Drive mInstance = new Drive();
    }

}
