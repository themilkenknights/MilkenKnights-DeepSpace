package frc.robot.lib.vision;

import frc.robot.Constants.VISION;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Translation2d;
import frc.robot.lib.util.InterpolatingDouble;

public class LimelightTarget {

	public static LimelightTarget EMPTY = new LimelightTarget(false, 0.0, 0.0, 0.0, 0.0, 0.0, new double[]{0, 0, 0, 0, 0, 0});
	private boolean validTarget;//Does limelight have a target
	private double xOffset; //Horizontal center of bounding box from center in degrees
	private double yOffset; //Vertical center of bounding box from center in degrees
	private double area; //Pixel area of bounding box
	private double captureTime; //Capture Time in Sec (Timer.FPGA)
	private double distance;
	private double[] poseArray;
	private Pose2d deltaPose;
	private double horizLength;
	private double vertLength;

	public LimelightTarget(boolean validTarget, double xOffset, double yOffset, double horizLength, double vertLength,
			double captureTime, double[] poseArray) {
		this.validTarget = validTarget;
		this.xOffset = xOffset;
		this.yOffset = yOffset;
		this.horizLength = horizLength;
		this.vertLength = vertLength;
		this.area = horizLength * vertLength;
		this.captureTime = captureTime;
		this.distance = VISION.kAreaToDistVisionMap.getInterpolated(new InterpolatingDouble(getArea())).value;
		this.poseArray = poseArray;
		this.deltaPose = new Pose2d(new Translation2d(-this.poseArray[2], this.poseArray[0]), Rotation2d.fromDegrees(this.poseArray[3]));
	}

	public double getHorizLength() {
		return horizLength;
	}

	public double getVertLength() {
		return vertLength;
	}

	public boolean isValidTarget() {
		return validTarget;
	}

	public double getXOffset() {
		return xOffset;
	}

	public double getYOffset() {
		return yOffset;
	}

	public double getArea() {
		return area;
	}

	public double getCaptureTime() {
		return captureTime;
	}

	public double getDistance() {
		return distance;
	}

	public Pose2d getDeltaPose() {
		return deltaPose;
	}

	public double[] getPoseArray() {
		return poseArray;
	}

	public String getPoseString() {
		return "X: " + poseArray[0] + " Y: " + poseArray[1] + " Z: " + poseArray[2] + " Yaw: " + poseArray[3] + " Pitch: " + poseArray[4] + " Roll: " + poseArray[5];
	}

	@Override
	public String toString() {
		return "X:" + xOffset + ", Y: " + yOffset + ", Area:" + area + ", Dt: " + captureTime + " Distance: " + distance;
	}
}

