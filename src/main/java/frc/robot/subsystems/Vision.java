package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode.CamMode;
import frc.robot.lib.vision.LimeLightControlMode.LedMode;
import frc.robot.lib.vision.LimeLightControlMode.StreamType;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.lib.vision.MkPixyTarget;

public class Vision extends Subsystem {


    private LimeLight mLimeLight;
    private NetworkTableEntry mLLX, mDist, mArea, mLED;
    private UsbCamera hatchCam;
    private int cameraStream = 0;
    private boolean isVision = false;
    private MjpegServer server;
    private HttpCamera LLFeed;

    private Vision() {
        hatchCam = CameraServer.getInstance().startAutomaticCapture(0);
        hatchCam.setConnectVerbose(0);

        LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");

        server = CameraServer.getInstance().addSwitchedCamera("Toggle Cam");
        server.setSource(LLFeed);
        server.setSource(hatchCam);

        ShuffleboardTab mVisionTab = Shuffleboard.getTab("Vision");
        mLLX = mVisionTab.add("Limelight X", 0.0).getEntry();
        mDist = mVisionTab.add("Limelight Dist", 0.0).getEntry();
        mArea = mVisionTab.add("Area", 0.0).getEntry();
        mLED = mVisionTab.add("LED State", true).getEntry();
        mLimeLight = new LimeLight();
        configDriverVision();

        mLimeLight.setLEDMode(LedMode.kforceOff);
    }

    public static Vision getInstance() {
        return Vision.InstanceHolder.mInstance;
    }

    @Override public void outputTelemetry(double timestamp) {
        mLLX.setDouble(mLimeLight.returnAverageTarget().getYaw());
        mDist.setDouble(mLimeLight.returnAverageTarget().getDistance());
        mArea.setDouble(mLimeLight.returnAverageTarget().getArea());
        mLED.setBoolean(mLimeLight.getLEDMode() != LedMode.kforceOff);
        SmartDashboard.putNumber("Pipeline", mLimeLight.getPipelineInt());
    }

    public void configLimelightVision() {
        mLimeLight.setPipeline(1);
        mLimeLight.setCamMode(CamMode.kvision);
        mLimeLight.setStream(StreamType.kStandard);
        mLimeLight.setLEDMode(LedMode.kforceOn);
        isVision = true;
    }

    public void configDriverVision() {
        mLimeLight.setPipeline(1);
        mLimeLight.setLEDMode(LedMode.kforceOff);
        isVision = false;
    }

    public void toggleVision() {
        if (isVision) {
            mLimeLight.setLEDMode(LedMode.kforceOff);
            isVision = false;
        } else {
            mLimeLight.setLEDMode(LedMode.kforceOn);
            isVision = true;
        }
    }

    public void teleopInit(double timestamp) {
        configDriverVision();
    }

    public void autonomousInit(double timestamp) {
        configDriverVision();
    }

    public void configHatchStream() {
       /* if (cameraStream != 0) {
            hatchCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            server.setSource(hatchCam);
            cargoCam.setConnectionStrategy(ConnectionStrategy.kForceClose);
            cameraStream = 0;
        } */
    }

    public void configCargoStream() {
    /*  if (cameraStream != 1) {
           cargoCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
            server.setSource(cargoCam);
            hatchCam.setConnectionStrategy(ConnectionStrategy.kForceClose);
            cameraStream = 1;
        } */
    }

    public void updateLimelight() {
        mLimeLight.getUpdate();
    }

    public MkPixyTarget getPixyTarget() {
        //TODO Fix
        // return mPixy.getLatestTarget();
        return new MkPixyTarget(0, 0, false, 0.0);
    }

    @Override public void onStop(double timestamp) {
        configDriverVision();
    }

    @Override public boolean checkSystem() {
        return mLimeLight.isConnected();
    }

    public synchronized LimelightTarget getLimelightTarget() {
        return mLimeLight.returnLastTarget();
    }

    private static class InstanceHolder {

        private static final Vision mInstance = new Vision();

    }
}
