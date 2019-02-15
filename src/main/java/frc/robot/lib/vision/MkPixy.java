package frc.robot.lib.vision;

import edu.wpi.first.wpilibj.Timer;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2.LinkType;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import java.util.ArrayList;

public class MkPixy {

	private static final int blockSignature = 1;
	private static ArrayList<Block> blocks = null;
	private static Pixy2 pixy = null;
	private static double lastUpdate = 0.0;
	//TODO Find a way of initializing block to non-null value
	private static Block mLastBlock = null;


	public MkPixy() {
		pixy = Pixy2.createInstance(LinkType.SPI);
		pixy.init();
	}

	public static synchronized void pixyUpdate() {
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
		return mLastBlock != null ? mLastBlock.getX() : 0.0;
	}

	public synchronized double getY() {
		return mLastBlock != null ? mLastBlock.getY() : 0.0;
	}

	public synchronized double getArea() {
		return mLastBlock != null ? mLastBlock.getHeight() * mLastBlock.getWidth() : 0.0;
	}

	public synchronized double getAngle() {
		return mLastBlock != null ? mLastBlock.getAngle() : 0.0;
	}

	public synchronized Block getLastBlock() {
		return mLastBlock;
	}

	public synchronized boolean isCargoIntaked() {
		return mLastBlock != null && getArea() > 45000;
	}

}
