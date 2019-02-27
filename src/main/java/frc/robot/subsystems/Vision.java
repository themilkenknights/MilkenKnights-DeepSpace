package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
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
    private MjpegServer server;
    private HttpCamera LLFeed;
    private UsbCamera cargoCam;
    private int cameraStream = 0;

    private Vision() {
        ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dash");
        LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
        cargoCam = CameraServer.getInstance().startAutomaticCapture(0);
        cargoCam.setConnectVerbose(0);
        server = CameraServer.getInstance().addSwitchedCamera("Toggle Cam");
        server.setSource(LLFeed);
        dashboardTab.add(server.getSource()).withWidget(BuiltInWidgets.kCameraStream).withPosition(1, 1).withSize(9, 4)
            .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify widget properties here
        Shuffleboard.selectTab("Drive");
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
        mLLX.setDouble(mLimeLight.returnAverageTarget().getYaw());
        mDist.setDouble(mLimeLight.returnAverageTarget().getDistance());
        mArea.setDouble(mLimeLight.returnAverageTarget().getArea());
    }

    private void configLimelightVision() {
        mLimeLight.setLEDMode(LedMode.kforceOn);
        mLimeLight.setCamMode(CamMode.kvision);
        mLimeLight.setStream(StreamType.kStandard);
    }

    public void teleopInit(double timestamp) {
        configLimelightVision();
    }

    public void autonomousInit(double timestamp) {
        configLimelightVision();
    }

    public void configHatchStream() {
        if (cameraStream != 0) {
            LLFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            server.setSource(LLFeed);
            cargoCam.setConnectionStrategy(ConnectionStrategy.kForceClose);
            cameraStream = 0;
        }
    }

    public void configCargoStream() {
        if (cameraStream != 1) {
            cargoCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            server.setSource(cargoCam);
            LLFeed.setConnectionStrategy(ConnectionStrategy.kForceClose);
            cameraStream = 1;
        }
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
