package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.RobotState.DriveControlState;
import frc.robot.util.drivers.MkJoystick;
import frc.robot.util.drivers.MkJoystickButton;
import frc.robot.util.math.DriveHelper;
import frc.robot.util.state.DriveSignal;
import frc.robot.util.structure.Subsystem;
import frc.robot.util.structure.loops.Loop;
import frc.robot.util.structure.loops.Looper;

public class Input extends Subsystem {

    private final MkJoystick operatorJoystick = new MkJoystick(1);

    private final MkJoystick driverJoystick = new MkJoystick(0);
    private final MkJoystickButton changeDriveMode = driverJoystick.getButton(1, "Change Drive Mode");
    private final MkJoystickButton toggleLEDSignal = driverJoystick.getButton(2, "Toggle HP Signal");
    private final MkJoystickButton turnOffLED = driverJoystick.getButton(3, "Turn Off LED");


    public Input() {

    }

    public static Input getInstance() {
        return InstanceHolder.mInstance;
    }

    @Override
    public void outputToSmartDashboard() {
        SmartDashboard.putNumber("Drive Command",-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3));
    }

    @Override
    public void checkSystem() {

    }

    @Override
    public void registerEnabledLoops(Looper enabledLooper) {
        Loop mLoop = new Loop() {

            @Override
            public void onStart(double timestamp) {
                synchronized (Input.this) {

                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Input.this) {
                    if (RobotState.mMatchState.equals(RobotState.MatchState.TELEOP)) {
                        updateDriveInput();
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        };
        enabledLooper.register(mLoop);
    }

    private void updateDriveInput() {
      /*  if (changeDriveMode.isPressed()) {
            Drive.getInstance().configVelocityControl();
            RobotState.mDriveControlState =
                    RobotState.mDriveControlState.equals(DriveControlState.OPEN_LOOP)
                            ? DriveControlState.VELOCITY_SETPOINT : DriveControlState.OPEN_LOOP;
        } */
        DriveSignal sig = DriveHelper
                .cheesyDrive((-driverJoystick.getRawAxis(2) + driverJoystick.getRawAxis(3)),
                        (-driverJoystick.getRawAxis(0)), true);
        if (RobotState.mDriveControlState == DriveControlState.VELOCITY_SETPOINT) {
            Drive.getInstance().setVelocitySetpoint(sig, 0, 0);
        } else if (RobotState.mDriveControlState == DriveControlState.OPEN_LOOP) {
            Drive.getInstance().setOpenLoop(sig);
        }

        if (toggleLEDSignal.isPressed()) {
            Superstructure.getInstance().toggleSignal();
        }
        if (turnOffLED.isPressed()) {
            Superstructure.getInstance().toggleLEDOff();
        }
    }

    private static class InstanceHolder {

        private static final Input mInstance = new Input();
    }

}
