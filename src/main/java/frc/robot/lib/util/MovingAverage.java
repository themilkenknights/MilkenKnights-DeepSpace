package frc.robot.lib.util;

import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.vision.LimelightTarget;
import java.util.ArrayList;

/**
 * Helper class for storing and calculating a moving average
 */
public class MovingAverage {

	ArrayList<LimelightTarget> targets = new ArrayList<LimelightTarget>();
	int maxSize;

	public MovingAverage(int maxSize) {
		this.maxSize = maxSize;
	}

	public void addNumber(LimelightTarget newNumber) {
		targets.add(newNumber);
		if (targets.size() > maxSize) {
			targets.remove(0);
		}
	}

	public LimelightTarget getAverage() {
		boolean validTarget = true;
		double totalX = 0;
		double totalY = 0;
		double totalArea = 0;
		double totalCaptureTime = 0;

		for (LimelightTarget target : targets) {
			validTarget = target.isValidTarget() && validTarget;
			totalX += target.getXOffset();
			totalY += target.getYOffset();
			totalArea += target.getArea();
			totalCaptureTime += target.getCaptureTime();
		}

		LimelightTarget avgTarget = new LimelightTarget(validTarget, totalX / targets.size(),
				totalY / targets.size(), totalArea / targets.size(),
				totalCaptureTime / targets.size(), Pose2d.identity());

		return avgTarget;
	}

	public boolean isUnderMaxSize() {
		return getSize() < maxSize;
	}

	public int getSize() {
		return targets.size();
	}

	public void clear() {
		targets.clear();
	}
}
