package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode.CamMode;
import frc.robot.lib.vision.LimeLightControlMode.LedMode;
import frc.robot.lib.vision.LimeLightControlMode.StreamType;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.lib.vision.MkPixy;
import frc.robot.lib.vision.MkPixyTarget;
import java.util.Map;

public class Vision extends Subsystem {

    private static MkPixy mPixy;
    private LimeLight mLimeLight;
    private boolean usePixy = false;
    private NetworkTableEntry mLLX, mDist, mArea;

    private Vision() {
        ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dash");

        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        dashboardTab.add(camera).withWidget(BuiltInWidgets.kCameraStream).withPosition(1, 1).withSize(4,4)
            .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify widget properties here

        HttpCamera httpCamera = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
        CameraServer.getInstance().addCamera(httpCamera);
        dashboardTab.add(httpCamera).withWidget(BuiltInWidgets.kCameraStream).withPosition(5, 1).withSize(5,4)
            .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify widget properties here

        Shuffleboard.selectTab("Dash");
        //TODO Fix Shuffleboard Stream Config

        ShuffleboardTab mVisionTab = Shuffleboard.getTab("Vision");
        mLLX = mVisionTab.add("Limelight X", 0.0).getEntry();
        mDist = mVisionTab.add("Limelight Dist", 0.0).getEntry();
        mArea = mVisionTab.add("Area", 0.0).getEntry();
        mLimeLight = new LimeLight();
        configLimelightVision();
        mPixy = new MkPixy();
    }

    public static Vision getInstance() {
        return Vision.InstanceHolder.mInstance;
    }

    @Override public void outputTelemetry(double timestamp) {
        if (usePixy) {
            SmartDashboard.putNumber("Pixy Yaw", mPixy.getLatestTarget().getYaw());
            SmartDashboard.putNumber("Pixy Area", mPixy.getLatestTarget().getArea());
            SmartDashboard.putBoolean("Pixy Limit Switch", mPixy.getLatestTarget().isCargoIntaked());
            SmartDashboard.putBoolean("Pixy State", mPixy.getLatestTarget().isTargetAcquired());
        }
        mLLX.setDouble(mLimeLight.returnAverageTarget().getXOffset());
        mDist.setDouble(mLimeLight.returnAverageTarget().getDistance());
        mArea.setDouble(mLimeLight.returnAverageTarget().getArea());
    }

    private void configLimelightVision() {
        mLimeLight.setLEDMode(LedMode.kforceOn);
        mLimeLight.setCamMode(CamMode.kvision);
        mLimeLight.setStream(StreamType.kPiPMain);
    }

    public void teleopInit(double timestamp) {
        configLimelightVision();
    }

    public void autonomousInit(double timestamp) {
        configLimelightVision();
    }

    public void updateLimelight() {
        mLimeLight.getUpdate();
    }

    public void updatePixy() {
        mPixy.pixyUpdate();
    }

    public MkPixyTarget getPixyTarget() {
        return mPixy.getLatestTarget();
    }

    @Override public void onStop(double timestamp) {

    }

    @Override public boolean checkSystem() {
        return mLimeLight.isConnected();
    }

    public synchronized LimelightTarget getLimelightTarget() {
        return mLimeLight.returnAverageTarget();
    }

    private static class InstanceHolder {

        private static final Vision mInstance = new Vision();

    }
}
