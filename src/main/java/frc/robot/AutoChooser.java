package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.lib.math.trajectory.Path;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MatchData;
import frc.robot.subsystems.Superstructure;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import java.util.HashMap;
import java.util.Map;

public class AutoChooser {
  public static final Trajectory.Config veryFastConfig = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_FAST,
      0.01, 130, 110, 5000);

  public static final Map<String, Path> autoPaths = new HashMap<>();
  public static MatchData matchData = MatchData.defaultMatch;
  public static AutoPosition mAutoPosition = AutoPosition.RIGHT;
  private static AutoModeExecutor mAutoModeExecuter;
  private static SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
  private static ShuffleboardTab mTab = Shuffleboard.getTab("General");
  private static ComplexWidget positionChooserTab = mTab.add("Position", positionChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);

  public synchronized static void startAuto(AutoModeBase base) {
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

  public static void autoInit() {
    if (mAutoPosition == AutoPosition.LEFT || mAutoPosition == AutoPosition.RIGHT) {
      Superstructure.getInstance().setRobotState(Superstructure.RobotState.CARGOSHIP_AUTO);
    } else {
      Logger.logMarker("Not Running Auto");
    }
  }

  /*
  Initialize Smart Dashboard Chooser
   */
  public static void loadAutos() {
    positionChooser.setDefaultOption("Nothing", AutoPosition.NOTHING);
    positionChooser.addOption("Left", AutoPosition.LEFT);
    positionChooser.addOption("Right", AutoPosition.RIGHT);
    loadPaths();
  }

  public static Path getPath(String name) {
    try {
      return autoPaths.get(name);
    } catch (NullPointerException e) {
      Logger.logErrorWithTrace("Failed to get path");
      return new Path();
    }
  }

  public static void loadPaths() {
    HashMap<String, PathContainer> robotPaths = new HashMap<>();

    robotPaths.put("CS-1", new PathContainer(new Waypoint[] {
        new Waypoint(204, -12, Pathfinder.d2r(0)),
        new Waypoint(175, 19, Pathfinder.d2r(90)),
    }, veryFastConfig));

    robotPaths.put("CS-2", new PathContainer(new Waypoint[] {
        new Waypoint(175, 19, Pathfinder.d2r(90)),
        new Waypoint(70, -136, Pathfinder.d2r(0)),
        new Waypoint(32, -136, Pathfinder.d2r(0)),
    }, veryFastConfig));

    robotPaths.put("CS-3", new PathContainer(new Waypoint[] {
        new Waypoint(20, -136, Pathfinder.d2r(0)),
        new Waypoint(234, -62, Pathfinder.d2r(5)),
        new Waypoint(264, -86, Pathfinder.d2r(95)),
    }, veryFastConfig));

    double tiL = 0;
    double tiR = 0;
    for (Map.Entry<String, PathContainer> container : robotPaths.entrySet()) {
      PathContainer traj = container.getValue();
      Trajectory leftTraj = Pathfinder.generate(traj.getPoints(), traj.getConfig());
      Trajectory rightTraj = Pathfinder.generate(traj.getRightPoints(), traj.getConfig());

      TankModifier lmodifier = new TankModifier(leftTraj).modify(Constants.DRIVE.kEffectiveDriveWheelTrackWidthInches);
      Trajectory leftl = lmodifier.getLeftTrajectory();
      Trajectory rightl = lmodifier.getRightTrajectory();

      TankModifier rmodifier = new TankModifier(rightTraj).modify(Constants.DRIVE.kEffectiveDriveWheelTrackWidthInches);
      Trajectory leftr = rmodifier.getLeftTrajectory();
      Trajectory rightr = rmodifier.getRightTrajectory();

      autoPaths.put(container.getKey() + "L", new Path(container.getKey() + "L", new Path.Pair(leftl, rightl)));
      autoPaths.put(container.getKey() + "R", new Path(container.getKey() + "R", new Path.Pair(leftr, rightr)));

      //System.out.println("Path: " + container.getKey() + " Time: " + leftTraj.length() * 0.01 + " Sec");
      if (container.getKey().charAt(0) == 'C') {
        tiL += leftTraj.length() * 0.01;
        tiR += rightTraj.length() * 0.01;
      }
    }
    System.out.println("Left: " + tiL + " Right: " + tiR);
  }

  public enum AutoPosition {
    LEFT, NOTHING, RIGHT
  }
}

class PathContainer {

  Waypoint[] points;
  Trajectory.Config config;
  boolean first;

  public PathContainer(Waypoint[] points, Trajectory.Config config, boolean first, boolean bothSides) {
    this.points = points;
    this.config = config;
    this.first = first;
  }

  public PathContainer(Waypoint[] points, Trajectory.Config config) {
    this(points, config, false, true);
  }

  public Waypoint[] getPoints() {
    return points;
  }

  public Waypoint[] getRightPoints() {
    Waypoint[] waypoints = new Waypoint[points.length];
    for (int i = 0; i < waypoints.length; i++) {
      waypoints[i] = new Waypoint(points[i].x, points[i].y, points[i].angle);
    }
    if (first) {
      for (int i = 1; i < waypoints.length; i++) {
        waypoints[i].y *= -1.0;
        waypoints[i].angle *= -1.0;
      }
    } else {
      for (Waypoint waypoint : waypoints) {
        waypoint.y *= -1.0;
        waypoint.angle *= -1.0;
      }
    }
    return waypoints;
  }

  public Trajectory.Config getConfig() {
    return config;
  }
}
