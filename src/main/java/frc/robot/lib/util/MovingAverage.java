package frc.robot.lib.util;

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
		double totalHoriz = 0;
		double totalVert = 0;
		double totalCaptureTime = 0;
		double[] poseArray = new double[]{0, 0, 0, 0, 0, 0};

		for (LimelightTarget target : targets) {
			validTarget = target.isValidTarget() && validTarget;
			totalX += target.getXOffset();
			totalY += target.getYOffset();
			totalHoriz += target.getArea();
			totalVert += target.getArea();
			totalCaptureTime += target.getCaptureTime();
			for (int i = 0; i < poseArray.length; i++) {
				poseArray[i] = target.getPoseArray()[i];
			}
		}
		double[] finalPoseArray = new double[6];
		for (int i = 0; i < poseArray.length; i++) {
			finalPoseArray[i] = poseArray[i] / targets.size();
		}
		LimelightTarget avgTarget = new LimelightTarget(validTarget, totalX / targets.size(),
				totalY / targets.size(), totalHoriz / targets.size(), totalVert / targets.size(),
				totalCaptureTime / targets.size(), finalPoseArray);
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
