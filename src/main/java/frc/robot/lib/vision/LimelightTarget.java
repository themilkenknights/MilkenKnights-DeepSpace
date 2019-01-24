package frc.robot.lib.vision;

public class LimelightTarget {
    public static LimelightTarget EMPTY = new LimelightTarget(false, 0.0, 0.0, 0.0, 0.0, 0.0);
    private boolean validTarget;//Does limelight have a target
    private double xOffset; //Horizontal center of bounding box from center in degrees
    private double yOffset; //Vertical center of bounding box from center in degrees
    private double area; //Pixel area of bounding box
    private double captureTime; //Capture Time in Sec (Timer.FPGA)

    public LimelightTarget(boolean validTarget, double xOffset, double yOffset, double horizLength, double vertLength, double captureTime) {
        this.validTarget = validTarget;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.area = horizLength * vertLength;
        this.captureTime = captureTime;
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

    @Override
    public String toString() {
        return "X:" + xOffset + ", Y: " + yOffset + ", Area:" + area + ", Dt: " + captureTime;
    }
}

