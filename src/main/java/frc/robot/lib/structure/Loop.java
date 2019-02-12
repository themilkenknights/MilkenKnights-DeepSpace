package frc.robot.lib.structure;

/**
 * Interface for loops, which are routine that run periodically in the robot code (such as periodic gyroscope calibration, etc.)
 */
public interface Loop {

	void onLoop(double timestamp);
}
