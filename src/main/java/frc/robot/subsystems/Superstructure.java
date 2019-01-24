package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.structure.ILooper;
import frc.robot.lib.structure.Loop;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive.DriveControlState;

public class Superstructure extends Subsystem {
    private LimeLight limeLight;
    private LimelightTarget target;

    private Superstructure() {
        limeLight = new LimeLight();
        target = LimelightTarget.EMPTY;
    }

    public static Superstructure getInstance() {
        return InstanceHolder.mInstance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Robot State", Robot.mMatchState.toString());
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
                synchronized (Superstructure.this) {
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                  
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    public synchronized LimelightTarget getTarget() {
        return limeLight.returnTarget();
    }

    private static class InstanceHolder {
        private static final Superstructure mInstance = new Superstructure();
    }
}
