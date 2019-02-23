package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoSource;
import edu.wpi.cscore.VideoSource.Kind;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode.CamMode;
import frc.robot.lib.vision.LimeLightControlMode.LedMode;
import frc.robot.lib.vision.LimeLightControlMode.StreamType;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.lib.vision.MkPixy;

public class Vision extends Subsystem {

	public static LimeLight mLimeLight;
	public static MkPixy mPixy;
	private boolean usePixy = false;
	private ShuffleboardTab mVisionTab;
	private NetworkTableEntry mLLX, mDist, mArea;

	private Vision() {
		//UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		mVisionTab = Shuffleboard.getTab("Vision");
		mLLX = mVisionTab.add("Limelight X", 0.0).getEntry();
		mDist = mVisionTab.add("Limelight Dist", 0.0).getEntry();
		mArea = mVisionTab.add("Area", 0.0).getEntry();
		//mVisionTab.add(camera).withWidget(BuiltInWidgets.kCameraStream).withSize(2,2);
		HttpCamera httpCamera = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
		CameraServer.getInstance().addCamera(httpCamera);
		mVisionTab.add(httpCamera).withWidget(BuiltInWidgets.kCameraStream).withSize(2,2);
		//TODO Fix Shuffleboard Stream Config
		mLimeLight = new LimeLight();
		mLimeLight.setLEDMode(LedMode.kforceOn);
		mLimeLight.setCamMode(CamMode.kvision);
		mLimeLight.setStream(StreamType.kPiPMain);
		mPixy = new MkPixy();
	}

	public static Vision getInstance() {
		return Vision.InstanceHolder.mInstance;
	}

	@Override
	public void outputTelemetry(double timestamp) {
		if (usePixy) {
			SmartDashboard.putNumber("Pixy X", mPixy.getX());
			SmartDashboard.putNumber("Pixy Y", mPixy.getY());
			SmartDashboard.putNumber("Pixy Area", mPixy.getArea());
			SmartDashboard.putBoolean("Pixy Limit Switch", mPixy.isCargoIntaked());
		}
		mLLX.setDouble(mLimeLight.returnAverageTarget().getXOffset());
		mDist.setDouble(mLimeLight.returnAverageTarget().getDistance());
		mArea.setDouble(mLimeLight.returnAverageTarget().getArea());
	}

	public void teleopInit(double timestamp) {
		mLimeLight.setLEDMode(LedMode.kforceOn);
	}

	public void autonomousInit(double timestamp) {
		mLimeLight.setLEDMode(LedMode.kforceOn);
	}

	@Override
	public void onMainLoop(double timestamp) {

	}

	@Override
	public void onStop(double timestamp) {

	}

	@Override
	public boolean checkSystem() {
		return mLimeLight.isConnected();
	}

	public synchronized LimelightTarget getAverageTarget() {
		return mLimeLight.returnLastTarget();
	}

	private static class InstanceHolder {

		private static final Vision mInstance = new Vision();

	}
}
