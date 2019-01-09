package frc.robot;

import frc.robot.lib.util.MatchData;
import frc.robot.subsystems.Drive.DriveControlState;

public class ControlState {

	public static MatchData matchData = MatchData.defaultMatch;

	public static void resetDefaultState() {

		ControlState.matchData = MatchData.defaultMatch;
	}



}
