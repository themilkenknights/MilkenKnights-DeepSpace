package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.MkTime;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode.LedMode;
import frc.robot.lib.vision.LimelightTarget;

public class Vision extends Subsystem {


    private LimeLight mLimeLight;
    private NetworkTableEntry mLLX, mDist, mArea, mLED;
    private UsbCamera cargoCam;
    private boolean isVision = false;
    private MjpegServer server;
    private HttpCamera LLFeed;
    private MkTime mLEDTimer = new MkTime();
    private MkTime autoOffTimer = new MkTime();

    private Vision() {
        cargoCam = CameraServer.getInstance().startAutomaticCapture(0);
        cargoCam.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 30);
        cargoCam.setConnectVerbose(0);

        LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");

        server = new MjpegServer("Switched Camera", 5805);
        server.setSource(LLFeed);
        server.setSource(cargoCam);

        ShuffleboardTab mVisionTab = Shuffleboard.getTab("Vision");
        mLLX = mVisionTab.add("Limelight X", 0.0).getEntry();
        mDist = mVisionTab.add("Limelight Dist", 0.0).getEntry();
        mArea = mVisionTab.add("Area", 0.0).getEntry();
        mLED = mVisionTab.add("LED State", true).getEntry();
        mLimeLight = new LimeLight();
        disableLED();
    }

    public static Vision getInstance() {
        return Vision.InstanceHolder.mInstance;
    }

    @Override public void outputTelemetry(double timestamp) {
        mLLX.setDouble(mLimeLight.returnAverageTarget().getYaw());
        mDist.setDouble(mLimeLight.returnAverageTarget().getDistance());
        mArea.setDouble(mLimeLight.returnAverageTarget().getArea());
        mLED.setBoolean(mLimeLight.getLEDMode() != LedMode.kforceOff);
    }

    public void setVisionPipeline() {
        mLimeLight.setPipeline(0);
        mLimeLight.setLEDMode(LedMode.kforceOff);
        isVision = false;
    }

    public void toggleVision() {
        if (isVision) {
            disableLED();
        } else {
            enableLED();
        }
    }

    public void enableLED() {
        if (!isVision) {
            mLimeLight.setLEDMode(LedMode.kforceOn);
            isVision = true;
            mLEDTimer.start(0.05);
        }
    }

    public boolean timerDone() {
        return mLEDTimer.isDone();
    }

    public void disableLED() {
        if (isVision) {
            mLimeLight.setLEDMode(LedMode.kforceOff);
            isVision = false;
            mLEDTimer.reset();
        }
    }

    public void teleopInit(double timestamp) {
        setVisionPipeline();
    }

    public void autonomousInit(double timestamp) {
        setVisionPipeline();
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

    public synchronized void updateLimelight() {
        mLimeLight.getUpdate();
    }

    @Override public void onStop(double timestamp) {
        setVisionPipeline();
    }

    @Override public void onRestart(double timestamp) {

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
