package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.MovingAverage;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode;
import frc.robot.lib.vision.LimeLightControlMode.CamMode;
import frc.robot.lib.vision.LimeLightControlMode.LedMode;
import frc.robot.lib.vision.LimeLightControlMode.StreamType;
import frc.robot.lib.vision.LimelightTarget;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import java.util.ArrayList;

public class Vision extends Subsystem {

	private static final int blockSignature = 1;
	private static ArrayList<Block> blocks = null;
	private LimeLight limeLight;
	private MovingAverage mThrottleAverage = new MovingAverage(3);
	private boolean usePixy = false;
	private Pixy2 pixy = null;
	private CamMode mCurrentCamMode = CamMode.kvision;
	private StreamType mCurrentStreamType = StreamType.kPiPMain;

	private Vision() {
		limeLight = new LimeLight();
		limeLight.setLEDMode(LedMode.kforceOn);
		limeLight.setCamMode(CamMode.kvision);
		limeLight.setStream(StreamType.kPiPMain);
		if (usePixy) {
			pixy = Pixy2.createInstance(LinkType.SPI);
			pixy.init();
		}
	}

	public static Vision getInstance() {
		return Vision.InstanceHolder.mInstance;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putNumber("LL Area", mThrottleAverage.getAverage().getArea());
		SmartDashboard.putNumber("LL X", mThrottleAverage.getAverage().getXOffset());
		SmartDashboard.putNumber("LL Dist", mThrottleAverage.getAverage().getDistance());
		if (usePixy) {
			pixyUpdate();
			System.out.println("Test");
		}
	}

	public void teleopInit(double timestamp) {
		limeLight.setLEDMode(LedMode.kforceOn);
	}

	public void autonomousInit(double timestamp) {
		limeLight.setLEDMode(LedMode.kforceOn);
	}

	public void setStreamMode(StreamType streamMode) {
		if (mCurrentStreamType != streamMode) {
			limeLight.setStream(streamMode);
			mCurrentStreamType = streamMode;
		}
	}

	public void setCamMode(CamMode camMode) {
		if (mCurrentCamMode != camMode) {
			limeLight.setCamMode(camMode);
			mCurrentCamMode = camMode;
		}
	}

	public Pose2d getTargetTranslation() {
		return limeLight.returnTarget().getDeltaPose();
	}

	@Override
	public void onLoop(double timestamp) {
		//limeLight.returnTarget();
		//mThrottleAverage.addNumber(limeLight.returnTarget());
		//System.out.println(mThrottleAverage.getAverage().getDistance());
	}

	public void pixyUpdate() {
		pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
		blocks = pixy.getCCC().getBlocks();
		Block largestBlock = null;
		for (Block block : blocks) {
			if (block.getSignature() == blockSignature) {
				if (largestBlock == null) {
					largestBlock = block;
				} else if (block.getWidth() > largestBlock.getWidth()) {
					largestBlock = block;
				}
			}
		}
		System.out.println(blocks);
	}

	@Override
	public void onStop(double timestamp) {
		mThrottleAverage.clear();
		limeLight.setLEDMode(LimeLightControlMode.LedMode.kforceOff);
	}

	@Override
	public boolean checkSystem() {
		return limeLight.isConnected();
	}

	public synchronized LimelightTarget getAverageTarget() {
		return mThrottleAverage.getAverage();
	}

	private static class InstanceHolder {

		private static final Vision mInstance = new Vision();
	}
}
