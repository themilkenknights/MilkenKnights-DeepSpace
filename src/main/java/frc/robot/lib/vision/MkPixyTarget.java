package frc.robot.lib.vision;

public class MkPixyTarget {

	private double yaw;
	private double area;
	private boolean isCargoIntaked;
	private boolean targetAcquired;
	private double timestamp;

	public MkPixyTarget(double x, double area, boolean targetAcquired, double timestamp) {
		yaw = (x - 157.5) * 0.1904761905;
		this.area = area;
		this.targetAcquired = targetAcquired;
		isCargoIntaked = this.area > 45000;
		this.timestamp = timestamp;
	}

	public double getYaw() {
		return yaw;
	}

	public double getArea() {
		return area;
	}

	public boolean isCargoIntaked() {
		return isCargoIntaked;
	}

	public boolean isTargetAcquired() {
		return targetAcquired;
	}

	public double getTimestamp() {
		return timestamp;
	}

}
