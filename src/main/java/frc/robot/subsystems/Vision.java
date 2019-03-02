package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
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

   // private static MkPixy mPixy;
    private LimeLight mLimeLight;
    private boolean usePixy = false;
    private NetworkTableEntry mLLX, mDist, mArea, mLED;
  //  private MjpegServer server;
    //private HttpCamera LLFeed;
    private UsbCamera hatchCam;
    //private UsbCamera cargoCam;
    private int cameraStream = 0;
    private boolean isVision = false;

    private Vision() {
        //ShuffleboardTab dashboardTab = Shuffleboard.getTab("Dash");
        //LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
       // cargoCam = CameraServer.getInstance().startAutomaticCapture(1);
       // cargoCam.setVideoMode(PixelFormat.kMJPEG, 320, 240, 20);
       // cargoCam.setConnectVerbose(0);
        hatchCam = CameraServer.getInstance().startAutomaticCapture(0);
      //  hatchCam.setVideoMode(PixelFormat.kMJPEG, 320, 240, 20);
        hatchCam.setConnectVerbose(0);
       // server = CameraServer.getInstance().addSwitchedCamera("Toggle Cam");
        //server.setSource(cargoCam);
        //server.setSource(hatchCam);
      /*  dashboardTab.add(server.getSource()).withWidget(BuiltInWidgets.kCameraStream).withPosition(1, 1).withSize(9, 4)
            .withProperties(Map.of("Show Crosshair", true, "Show Controls", false));// specify widget properties here

        Shuffleboard.selectTab("Dash");*/
        ShuffleboardTab mVisionTab = Shuffleboard.getTab("Vision");
        mLLX = mVisionTab.add("Limelight X", 0.0).getEntry();
        mDist = mVisionTab.add("Limelight Dist", 0.0).getEntry();
        mArea = mVisionTab.add("Area", 0.0).getEntry();
        mLED = mVisionTab.add("LED State", true).getEntry();
        mLimeLight = new LimeLight();
        configDriverVision();
       // mPixy = new MkPixy();
        mLimeLight.setLEDMode(LedMode.kforceOff);
    }

    public static Vision getInstance() {
        return Vision.InstanceHolder.mInstance;
    }

    @Override public void outputTelemetry(double timestamp) {
       /* if (usePixy) {
            SmartDashboard.putNumber("Pixy Yaw", mPixy.getLatestTarget().getYaw());
            SmartDashboard.putNumber("Pixy Area", mPixy.getLatestTarget().getArea());
            SmartDashboard.putBoolean("Pixy Limit Switch", mPixy.getLatestTarget().isCargoIntaked());
            SmartDashboard.putBoolean("Pixy State", mPixy.getLatestTarget().isTargetAcquired());
        } */
        mLLX.setDouble(mLimeLight.returnAverageTarget().getYaw());
        mDist.setDouble(mLimeLight.returnAverageTarget().getDistance());
        mArea.setDouble(mLimeLight.returnAverageTarget().getArea());
        mLED.setBoolean(mLimeLight.getLEDMode() != LedMode.kforceOff);
        SmartDashboard.putNumber("Pipeline", mLimeLight.getPipelineInt());
    }

    public void configLimelightVision() {
        mLimeLight.setPipeline(1);
        mLimeLight.setLEDMode(LedMode.kforceOn);
        isVision = true;
       //TODO Enable  mLimeLight.setCamMode(CamMode.kvision);
       // mLimeLight.setStream(StreamType.kStandard);
    }

    public void configDriverVision(){
        mLimeLight.setPipeline(1);
        mLimeLight.setLEDMode(LedMode.kforceOff);
     isVision = false;
    }

    public void toggleVision(){
        if(isVision){
            mLimeLight.setLEDMode(LedMode.kforceOff);
            isVision = false;
        } else{
            mLimeLight.setLEDMode(LedMode.kforceOn);
            isVision = true;
        }
    }

    public void teleopInit(double timestamp) {
        mLimeLight.setPipeline(1);
        mLimeLight.setLEDMode(LedMode.kforceOff);
        isVision = false;
    }

    public void autonomousInit(double timestamp) {
        mLimeLight.setPipeline(1);
        mLimeLight.setLEDMode(LedMode.kforceOff);
        isVision = false;
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

    public void updatePixy() {
      //  mPixy.pixyUpdate();
    }

    public MkPixyTarget getPixyTarget() {
        //return mPixy.getLatestTarget();
        return new MkPixyTarget(0,0,false, 0.0);
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
