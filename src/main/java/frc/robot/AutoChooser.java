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
  public static final Trajectory.Config fastConfig = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_FAST,
      0.01, 90, 90, 5000);

  public static final Trajectory.Config mediumConfig = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_LOW,
      0.01, 80, 50, 700);
  public static final Map<String, Path> autoPaths = new HashMap<>();
  public static final HashMap<String, OtherPath> robotPaths = new HashMap<>();
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

  /*
  Initialize Smart Dashboard Choosers and Serialize all paths into memory
   */
  public static void loadAutos() {
    positionChooser.setDefaultOption("Right", AutoPosition.RIGHT);
    positionChooser.addOption("Left", AutoPosition.LEFT);
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
    robotPaths.put("CS-1", new OtherPath(new Waypoint[] {
        new Waypoint(68 ,-47 ,Pathfinder.d2r(0)),
        new Waypoint(236 + 4 ,-102 - 12 ,Pathfinder.d2r(0)),
        new Waypoint(256 + 4 ,-78 - 12 ,Pathfinder.d2r(90)),
    }, veryFastConfig));

    robotPaths.put("CS-2", new OtherPath(new Waypoint[] {
        new Waypoint(261 ,-46 ,Pathfinder.d2r(90)),
        new Waypoint(261 ,-48 ,Pathfinder.d2r(90)),
        new Waypoint(271 ,-70 ,Pathfinder.d2r(135)),
    }, veryFastConfig));

    robotPaths.put("CS-3", new OtherPath(new Waypoint[] {
        new Waypoint(271 ,-70 ,Pathfinder.d2r(135)),
        new Waypoint(243 ,-60 - 14 ,Pathfinder.d2r(180)),
        new Waypoint(23 ,-135 - 14,Pathfinder.d2r(180)),
    }, fastConfig));

    robotPaths.put("CS-4", new OtherPath(new Waypoint[] {
        new Waypoint(20 ,-136 ,Pathfinder.d2r(10)),
        new Waypoint(260 ,-59 - 30 ,Pathfinder.d2r(5)),
        new Waypoint(285 ,-79 - 30 ,Pathfinder.d2r(95)),
    }, veryFastConfig));

    double tiL = 0;
    double tiR = 0;
    for (Map.Entry<String, OtherPath> container : robotPaths.entrySet()) {
      OtherPath traj = container.getValue();
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

        System.out.println("Path: " + container.getKey() + " Time: " + leftTraj.length() * 0.01 + " Sec");
        if (container.getKey().charAt(0) == 'C') {
          tiL += leftTraj.length() * 0.01;
          tiR += rightTraj.length() * 0.01;
      }
    }
    System.out.println("Left: " + tiL + " Right: " + tiR);
  }

  public enum AutoPosition {
    LEFT, CENTER, RIGHT
  }
}

class OtherPath {

  Waypoint[] points;
  Trajectory.Config config;
  boolean first;
  boolean bothSides;

  public OtherPath(Waypoint[] points, Trajectory.Config config, boolean first, boolean bothSides) {
    this.points = points;
    this.config = config;
    this.first = first;
    this.bothSides = bothSides;
  }

  public OtherPath(Waypoint[] points, Trajectory.Config config, boolean first) {
    this(points, config, first, true);
  }

  public OtherPath(Waypoint[] points, Trajectory.Config config) {
    this(points, config, false, true);
  }

  public void setOffset(double x, double y) {
    if (first) {
      for (int i = 1; i < points.length; i++) {
        points[i].x = points[i].x + x;
        points[i].y = points[i].y + y;
      }
    } else {
      for (Waypoint waypoint : points) {
        waypoint.x = waypoint.x + x;
        waypoint.y = waypoint.y + y;
      }
    }
  }

  public Waypoint[] getPoints() {
    return points;
  }

  public boolean isBothSides() {
    return bothSides;
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
