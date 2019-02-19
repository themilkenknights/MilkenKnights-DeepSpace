package frc.robot.lib.util;


import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class DeltaTime {

	private double initTime = 0.0;
	private int loopsTillUpdate, count;
	private NetworkTableEntry mEntry;

	public DeltaTime(String name, int loopsTillUpdate) {
		this.loopsTillUpdate = loopsTillUpdate;
		mEntry = Shuffleboard.getTab("Loops").add(name, 0.0).getEntry();
		count = 0;
	}

	public double start() {
		initTime = Timer.getFPGATimestamp();
		return initTime;
	}

	public void updateDt() {
		if (count == loopsTillUpdate) {
			mEntry.setDouble((Timer.getFPGATimestamp() - initTime) * 1e3);
			count = 0;
		}
		count++;
	}

}
