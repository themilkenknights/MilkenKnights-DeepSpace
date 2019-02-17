package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	private boolean usePixy = true;


	private Vision() {
		mLimeLight = new LimeLight();
		mLimeLight.setLEDMode(LedMode.kforceOn);
		mLimeLight.setCamMode(CamMode.kdriver);
		mLimeLight.setStream(StreamType.kStandard);
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
		SmartDashboard.putNumber("Limelight X", mLimeLight.returnAverageTarget().getXOffset());
		SmartDashboard.putNumber("Limelight Dist", mLimeLight.returnAverageTarget().getDistance());
		SmartDashboard.putNumber("Limelight Area", mLimeLight.returnAverageTarget().getArea());
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
		return mLimeLight.returnAverageTarget();
	}

	private static class InstanceHolder {

		private static final Vision mInstance = new Vision();

	}
}
