package frc.robot.lib.util;

import edu.wpi.first.wpilibj.Timer;

public class MkTime {

	private double timerLength = 0.0;
	private double startTime = 0.0;
	private boolean hasBeenSet = false;

	public void start(double timerLength) {
		this.timerLength = timerLength;
		startTime = Timer.getFPGATimestamp();
		hasBeenSet = true;
	}

	public void reset() {
		hasBeenSet = false;
	}

	public boolean hasBeenSet() {
		return hasBeenSet;
	}

	public boolean isDone() {
		return hasBeenSet && Timer.getFPGATimestamp() - startTime >= timerLength;
	}

}
