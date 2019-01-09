package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.CrossAutoLineMode;
import frc.robot.lib.util.CrashTracker;

public class AutoChooser {

	private static AutoModeExecutor mAutoModeExecuter = null;

	public static AutoModeBase getAutoMode() {
		double delay = SmartDashboard.getNumber("Auto Delay", 0.0);
		if (delay > 0) {
			Timer.delay(delay);
		}
		return getStraightMode();
	}

	private static AutoModeBase getStraightMode() {
		return new CrossAutoLineMode();
	}

	public static void startAuto() {
		updateGameData();
		if (mAutoModeExecuter != null) {
			mAutoModeExecuter.stop();
		}
		mAutoModeExecuter = null;
		mAutoModeExecuter = new AutoModeExecutor();
		mAutoModeExecuter.setAutoMode(getAutoMode());
		mAutoModeExecuter.start();
	}

	public static void disableAuto() {
		if (mAutoModeExecuter != null) {
			mAutoModeExecuter.stop();
		}
		mAutoModeExecuter = null;
	}

	private static void updateGameData() {
		ControlState.matchData.alliance = DriverStation.getInstance().getAlliance();
		ControlState.matchData.matchNumber = DriverStation.getInstance().getMatchNumber();
		ControlState.matchData.matchType = DriverStation.getInstance().getMatchType();
		CrashTracker.logMarker(
				"Alliance: " + ControlState.matchData.alliance.toString() + " Match Number: " + ControlState.matchData.matchNumber + " Match Type: "
						+ ControlState.matchData.matchType.toString());
	}

}
