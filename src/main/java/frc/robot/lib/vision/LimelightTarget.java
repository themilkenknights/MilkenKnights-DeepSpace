package frc.robot.lib.vision;

import frc.robot.Constants.VISION;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.util.InterpolatingDouble;

public class LimelightTarget {

	public static LimelightTarget EMPTY = new LimelightTarget(false, 0.0, 0.0, 0.0, 0.0, 0.0, Pose2d.identity());
	private boolean validTarget;//Does limelight have a target
	private double xOffset; //Horizontal center of bounding box from center in degrees
	private double yOffset; //Vertical center of bounding box from center in degrees
	private double area; //Pixel area of bounding box
	private double captureTime; //Capture Time in Sec (Timer.FPGA)
	private double distance;
	private Pose2d deltaPose;


	public LimelightTarget(boolean validTarget, double xOffset, double yOffset, double area,
			double captureTime, Pose2d deltaPose) {
		this.validTarget = validTarget;
		this.xOffset = xOffset;
		this.yOffset = yOffset;
		this.area = area;
		this.captureTime = captureTime;
		this.distance = VISION.kAreaToDistVisionMap.getInterpolated(new InterpolatingDouble(getArea())).value;
		this.deltaPose = deltaPose;
	}

	public LimelightTarget(boolean validTarget, double xOffset, double yOffset, double horizLength, double vertLength, double captureTime, Pose2d deltaPose) {
		this(validTarget, xOffset, yOffset, horizLength * vertLength, captureTime, deltaPose);
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

	@Override
	public String toString() {
		return "X:" + xOffset + ", Y: " + yOffset + ", Area:" + area + ", Dt: " + captureTime + " Distance: " + distance;
	}
}

