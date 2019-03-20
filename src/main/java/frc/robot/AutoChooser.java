package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MatchData;

public class AutoChooser {
  public static MatchData matchData = MatchData.defaultMatch;
  private static AutoModeExecutor mAutoModeExecuter = null;

  public static void startAuto(AutoModeBase base) {
    if (mAutoModeExecuter != null) {
      mAutoModeExecuter.stop();
    }
    mAutoModeExecuter = null;
    mAutoModeExecuter = new AutoModeExecutor();
    mAutoModeExecuter.setAutoMode(base);
    mAutoModeExecuter.start();
  }

  public static void updateGameData() {
    matchData.alliance = DriverStation.getInstance().getAlliance();
    matchData.matchNumber = DriverStation.getInstance().getMatchNumber();
    matchData.matchType = DriverStation.getInstance().getMatchType();
    Logger.logMarker("Alliance: " + matchData.alliance.toString() + " Match Number: " + matchData.matchNumber + " Match Type: " + matchData.matchType.toString());
  }

  public static void disableAuto() {
    if (mAutoModeExecuter != null) {
      mAutoModeExecuter.stop();
    }
    mAutoModeExecuter = null;
    mAutoModeExecuter = new AutoModeExecutor();
  }
}
