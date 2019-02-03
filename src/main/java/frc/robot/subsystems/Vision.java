package frc.robot.subsystems;

import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.MovingAverage;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode;
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

	private Vision() {

		limeLight = new LimeLight();
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

	}

	@Override
	public void zero(double timestamp) {
		limeLight.setLEDMode(LimeLightControlMode.LedMode.kforceOn);

	}

	@Override
	public void onLoop(double timestamp) {
		mThrottleAverage.addNumber(limeLight.returnTarget());
		if (usePixy) {
			pixyUpdate();
		}
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
		System.out.println(largestBlock.getX());
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
