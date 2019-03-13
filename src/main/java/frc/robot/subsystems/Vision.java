package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.MkTimer;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode.LedMode;
import frc.robot.lib.vision.LimeLightControlMode.StreamType;
import frc.robot.lib.vision.LimelightTarget;

public class Vision extends Subsystem {

  private LimeLight mLimeLight;
  private NetworkTableEntry mLLX, mDist, mArea, mLED, mSkew;
  // private UsbCamera cargoCam;
  private boolean isVision = false;
  // private MjpegServer server;
  // private HttpCamera LLFeed;
  private UsbCamera LLFeed;
  private boolean isHatchFeed = true;
  private MkTimer mLEDTimer = new MkTimer();

  private Vision() {
    // cargoCam = CameraServer.getInstance().startAutomaticCapture(0);
    // cargoCam.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 240, 30);
    // cargoCam.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 30);
    // cargoCam.setConnectVerbose(0);

    // LLFeed = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");

     LLFeed = CameraServer.getInstance().startAutomaticCapture(0);
     LLFeed.setVideoMode(PixelFormat.kMJPEG, 176,144,30);
    // LLFeed.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 240, 30);
    // LLFeed.setFPS(30);
    // LLFeed.setConnectVerbose(0);
    // server = new MjpegServer("Switched Camera", 5805);

    // server.setSource(cargoCam);
    // server.setSource(LLFeed);

    ShuffleboardTab mVisionTab = Shuffleboard.getTab("General");
    mLLX = mVisionTab.add("Limelight X", 0.0).getEntry();
    mDist = mVisionTab.add("Limelight Dist", 0.0).getEntry();
    mArea = mVisionTab.add("Limelight Area", 0.0).getEntry();
    mLED = mVisionTab.add("Limelight LED", true).getEntry();
    mSkew = mVisionTab.add("Limelight Skew", 0.0).getEntry();
    mLimeLight = new LimeLight();
  }

  public static Vision getInstance() {
    return Vision.InstanceHolder.mInstance;
  }

  @Override
  public void outputTelemetry(double timestamp) {
    mLLX.setDouble(getLimelightTarget().getYaw());
    mDist.setDouble(getLimelightTarget().getDistance());
    mArea.setDouble(getLimelightTarget().getArea());
    mLED.setBoolean(mLimeLight.getLEDMode() != LedMode.kforceOff);
    mSkew.setDouble(getLimelightTarget().getSkew());
  }

  public void teleopInit(double timestamp) {
    disableLED();
  }

  public void autonomousInit(double timestamp) {
    disableLED();
  }

  @Override
  public void onStop(double timestamp) {
    disableLED();
  }

  @Override
  public void onRestart(double timestamp) {
    disableLED();
  }

  @Override
  public boolean checkSystem() {
    return mLimeLight.isConnected();
  }

  public void disableLED() {
    mLimeLight.setStream(StreamType.kPiPSecondary);
    mLimeLight.setPipeline(0);
    if (isVision) {
      mLimeLight.setPipeline(0);
      isVision = false;
      mLEDTimer.reset();
    }
  }

  public synchronized LimelightTarget getLimelightTarget() {
    return mLimeLight.returnLastTarget();
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
      mLimeLight.setPipeline(0);
      isVision = true;
      mLEDTimer.start(0.2);
    }
  }

  public boolean timerDone() {
    return mLEDTimer.isDone();
  }

  public void configHatchStream() {
    /*
     * if (!isHatchFeed) { LLFeed.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
     * server.setSource(LLFeed); cargoCam.setConnectionStrategy(ConnectionStrategy.kForceClose);
     * isHatchFeed = true; }
     */
  }

  public void configCargoStream() {
    /*
     * if (isHatchFeed) { cargoCam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
     * server.setSource(cargoCam); LLFeed.setConnectionStrategy(ConnectionStrategy.kForceClose);
     * isHatchFeed = false; }
     */
  }

  public synchronized void updateLimelight() {
    mLimeLight.getUpdate();
  }

  private static class InstanceHolder {

    private static final Vision mInstance = new Vision();
  }
}
