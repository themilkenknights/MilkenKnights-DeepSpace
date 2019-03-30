package frc.robot.subsystems;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.lib.util.Subsystem;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode.LedMode;
import frc.robot.lib.vision.LimeLightControlMode.StreamType;
import frc.robot.lib.vision.LimelightTarget;

public class Vision extends Subsystem {
  private LimeLight mLimeLight;
  private NetworkTableEntry mLLX, mDist, mArea, mLED, mSkew, mValid;
  private boolean isVision;
  private UsbCamera hatchCam;
  private int i = 0;

  private Vision() {
    hatchCam = CameraServer.getInstance().startAutomaticCapture(0);
    hatchCam.setConnectVerbose(0);
    hatchCam.setVideoMode(PixelFormat.kMJPEG, 176, 144, 30);
    ShuffleboardTab mVisionTab = Shuffleboard.getTab("General");
    mLLX = mVisionTab.add("Limelight X", 0.0).getEntry();
    mDist = mVisionTab.add("Limelight Dist", 0.0).getEntry();
    mArea = mVisionTab.add("Limelight Area", 0.0).getEntry();
    mLED = mVisionTab.add("Limelight LED", true).getEntry();
    mSkew = mVisionTab.add("Limelight Skew", 0.0).getEntry();
    mValid = mVisionTab.add("Valid Target", false).getEntry();
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
    mValid.setBoolean(getLimelightTarget().isValidTarget());
  }

  public void teleopInit(double timestamp) {
    enableLED();
  }

  public void autonomousInit(double timestamp) {
    enableLED();
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
    //  if(isVision) {
    mLimeLight.setPipeline(1);
    isVision = false;
    // }
  }

  public void enableLED() {
    // if (!isVision) {
    ///  mLimeLight.setStream(StreamType.kStandard);
    mLimeLight.setPipeline(0);
    isVision = true;
    //  isVision = true;
    // }
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

  public synchronized void updateLimelight() {
    mLimeLight.getUpdate();
    if (i == 200) {
      i = 0;
      mLimeLight.setStream(StreamType.kPiPMain);
    }
    i++;
  }

  private static class InstanceHolder {
    private static final Vision mInstance = new Vision();
  }
}
