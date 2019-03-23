package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MatchData;
import frc.robot.lib.util.trajectory.Path;
import java.util.HashMap;
import java.util.Map;

public class AutoChooser {
  public static final Map<String, Path> autoPaths = new HashMap<>();
  public static MatchData matchData = MatchData.defaultMatch;
  public static AutoPosition mAutoPosition = AutoPosition.RIGHT;
  private static AutoModeExecutor mAutoModeExecuter;
  private static SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
  private static ShuffleboardTab mTab = Shuffleboard.getTab("General");
  private static ComplexWidget positionChooserTab = mTab.add("Position", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);

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
    mAutoPosition = positionChooser.getSelected();
    Logger.logMarker(
        "Alliance: " + matchData.alliance.toString() + " Match Number: " + matchData.matchNumber + " Match Type: " + matchData.matchType.toString());
  }

  public static void disableAuto() {
    if (mAutoModeExecuter != null) {
      mAutoModeExecuter.stop();
    }
    mAutoModeExecuter = null;
    mAutoModeExecuter = new AutoModeExecutor();
  }

  /*
  Initialize Smart Dashboard Choosers and Serialize all paths into memory
   */
  public static void loadAutos() {
    positionChooser.setDefaultOption("Right", AutoPosition.RIGHT);
    positionChooser.addOption("Left", AutoPosition.LEFT);
    /*for (String pathName : Constants.DRIVE.autoNames) {
      autoPaths.put(pathName + "L", DeserializePath.getPathFromFile(pathName + "L"));
      autoPaths.put(pathName + "R", DeserializePath.getPathFromFile(pathName + "R"));
    } */
  }

  public enum AutoPosition {
    LEFT, CENTER, RIGHT
  }
}
