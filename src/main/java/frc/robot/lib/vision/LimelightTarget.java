package frc.robot.lib.vision;

import frc.robot.Constants.VISION;
import frc.robot.lib.util.InterpolatingDouble;

public class LimelightTarget {

	public static LimelightTarget EMPTY = new LimelightTarget(false, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	private boolean validTarget; // Does limelight have a target
	private double xOffset; // Horizontal center of bounding box from center in degrees
	private double yOffset; // Vertical center of bounding box from center in degrees
	private double area; // Pixel area of bounding box
	private double captureTime; // Capture Time in Sec (Timer.FPGA)
	private double distance;
	private double horizLength;
	private double vertLength;
	private double skew;

	public LimelightTarget(boolean validTarget, double xOffset, double yOffset, double horizLength, double vertLength, double skew,
			double captureTime) {
		this.validTarget = validTarget;
		this.xOffset = xOffset;
		this.yOffset = yOffset;
		this.horizLength = horizLength;
		this.vertLength = vertLength;
		this.area = horizLength * vertLength;
		this.captureTime = captureTime;
		// this.distance = (24.0) / (Math.tan(Math.toRadians(35 + yOffset)));
		this.distance = VISION.kAreaToDistVisionMap.getInterpolated(new InterpolatingDouble(getArea())).value;
		this.skew = skew > -45.0 ? skew : skew + 90.0 ;
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

	public double getYaw() {
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

	public double getSkew() {
		return skew;
	}

	@Override
	public String toString() {
		return "X:" + xOffset + ", Y: " + yOffset + ", Area:" + area + ", Dt: " + captureTime + " Distance: " + distance + " Valid: "
				+ validTarget;
	}
}
