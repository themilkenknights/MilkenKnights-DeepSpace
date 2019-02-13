package frc.robot.lib.vision;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.util.Logger;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import java.util.ArrayList;

public class MkPixy {

	private static final int blockSignature = 1;
	private static ArrayList<Block> blocks = null;
	Notifier _pixyUpdate = new Notifier(new PeriodicRunnable());
	private Pixy2 pixy = null;
	private double mX, mY, mWidth, mHeight, lastUpdate = 0.0;
	//TODO Find a way of initializing block to non-null value
	private Block mLastBlock = null;


	public MkPixy() {
		pixy = Pixy2.createInstance(LinkType.SPI);
		pixy.init();
		_pixyUpdate.startPeriodic(GENERAL.kPixyLoopPeriod);
	}

	private void pixyUpdate() {
		pixy.getCCC().getBlocks(false, Pixy2CCC.CCC_SIG1, 5);
		blocks = pixy.getCCC().getBlocks();
		Block largestBlock = null;
		for (Block block : blocks) {
			if (block.getSignature() == blockSignature) {
				if (largestBlock == null) {
					largestBlock = block;
				} else if (block.getWidth() * block.getHeight() > largestBlock.getWidth() * largestBlock.getHeight()) {
					largestBlock = block;
				}
			}
		}
		if (largestBlock != null) {
			mLastBlock = largestBlock;
			lastUpdate = Timer.getFPGATimestamp();
		}
	}

	public synchronized double getX() {
		return mLastBlock.getX();
	}

	public synchronized double getY() {
		return mLastBlock.getY();
	}

	public synchronized double getArea() {
		return mLastBlock.getHeight() * mLastBlock.getWidth();
	}

	class PeriodicRunnable implements java.lang.Runnable {

		public void run() {
			try {
				pixyUpdate();
			} catch (Throwable t) {
				Logger.logThrowableCrash(t);
				throw t;
			}
		}
	}

}
