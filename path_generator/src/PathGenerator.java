import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Waypoint;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class PathGenerator {

  public static final HashMap<String, Path> robotPaths = new HashMap<>();
  public static final Trajectory.Config fastConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Config.SAMPLES_FAST, 0.02, 135, 80, 500);

  public static final double BLUE_LEFT_SWITCH_TO_SIDE_WALL = 0;
  public static final double BLUE_RIGHT_SWITCH_TO_SIDE_WALL = 0;
  public static final double BLUE_SWITCH_TO_WALL = 150;
  public static final double BLUE_SWITCH_X_OFFSET = BLUE_SWITCH_TO_WALL - 140;
  public static final double BLUE_SWITCH_Y_OFFSET =
      (BLUE_RIGHT_SWITCH_TO_SIDE_WALL - BLUE_LEFT_SWITCH_TO_SIDE_WALL) / 2;

  public static final double RED_LEFT_SWITCH_TO_SIDE_WALL = 0;
  public static final double RED_RIGHT_SWITCH_TO_SIDE_WALL = 0;
  public static final double RED_SWITCH_TO_WALL = 150;
  public static final double RED_SWITCH_X_OFFSET = RED_SWITCH_TO_WALL - 140;
  public static final double RED_SWITCH_Y_OFFSET =
      (RED_RIGHT_SWITCH_TO_SIDE_WALL - RED_LEFT_SWITCH_TO_SIDE_WALL) / 2;

  static {
   robotPaths.put("CS-2", new Path(new Waypoint[]{
        new Waypoint(0, 0, Pathfinder.d2r(0)),
        new Waypoint(35.5, 8.323, Pathfinder.d2r(23.343)),
    }, fastConfig));

    /* robotPaths.put("CS-3", new Path(new Waypoint[]{
        new Waypoint(48, 8, Pathfinder.d2r(-35)),
        new Waypoint(84, 0, Pathfinder.d2r(0)),
    }, defaultConfig));

    robotPaths.put("CS-4", new Path(new Waypoint[]{
        new Waypoint(84, 0, Pathfinder.d2r(0)),
        new Waypoint(62, 8, Pathfinder.d2r(-40)),
    }, fastConfig));

    robotPaths.put("CS-5", new Path(new Waypoint[]{
        new Waypoint(62, 8, Pathfinder.d2r(-40)),
        new Waypoint(121, -59, Pathfinder.d2r(0)),
    }, fastConfig));

    robotPaths.put("CS-6", new Path(new Waypoint[]{
        new Waypoint(121, -59, Pathfinder.d2r(0)),
        new Waypoint(63, 19, Pathfinder.d2r(-40)),
    }, fastConfig));

    robotPaths.put("CS-7", new Path(new Waypoint[]{
        new Waypoint(63, 19, Pathfinder.d2r(-40)),
        new Waypoint(96, 4, Pathfinder.d2r(0)),
    }, defaultConfig));

    robotPaths.put("CS-8", new Path(new Waypoint[]{
        new Waypoint(96, 4, Pathfinder.d2r(0)),
        new Waypoint(80, 5, Pathfinder.d2r(-30)),
    }, fastConfig));

    robotPaths.put("CS-9", new Path(new Waypoint[]{
        new Waypoint(80, 5, Pathfinder.d2r(-30)),
        new Waypoint(121, -49, Pathfinder.d2r(-15)),
    }, fastConfig));

    robotPaths.put("DriveStraight", new Path(new Waypoint[]{
        new Waypoint(23, 156, 0),
        new Waypoint(127, 156, 0)
    }, fastConfig));

    robotPaths.put("FS-1", new Path(new Waypoint[]{
        new Waypoint(22, -113, Pathfinder.d2r(0)),
        new Waypoint(119, -131, Pathfinder.d2r(0)),
        new Waypoint(163, -114, Pathfinder.d2r(45)),
        new Waypoint(170, -96, Pathfinder.d2r(90)),
    }, defaultConfig, false, true)); */

  }

  public static void main(String[] args) {
    double tiL = 0;
    double tiR = 0;
    for (Map.Entry<String, Path> container : robotPaths.entrySet()) {
        double t1 = System.nanoTime() * 1e-6;
        Trajectory trajectory = Pathfinder
            .generate(container.getValue().getPoints(), container.getValue().getConfig());
        System.out.println((System.nanoTime() * 1e-6) - t1);
      }
    }


  static class Path {

    Waypoint[] points;
    Trajectory.Config config;
    boolean first;
    boolean bothSides;

    public Path(Waypoint[] points, Trajectory.Config config, boolean first, boolean bothSides) {
      this.points = points;
      this.config = config;
      this.first = first;
      this.bothSides = bothSides;
    }

    public Path(Waypoint[] points, Trajectory.Config config, boolean first) {
      this(points, config, first, true);
    }

    public Path(Waypoint[] points, Trajectory.Config config) {
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

    public Waypoint[] getLeftPoints(double x, double y) {
      Waypoint[] waypoints = new Waypoint[points.length];
      for (int i = 0; i < waypoints.length; i++) {
        waypoints[i] = new Waypoint(points[i].x, points[i].y, points[i].angle);
      }
      if (first) {
        for (int i = 1; i < waypoints.length; i++) {
          waypoints[i].y *= -1.0;
          waypoints[i].angle *= -1.0;
          waypoints[i].x += x;
          waypoints[i].y += y;
        }
      } else {
        for (Waypoint waypoint : waypoints) {
          waypoint.y *= -1.0;
          waypoint.angle *= -1.0;
          waypoint.x += x;
          waypoint.y += y;
        }
      }
      return waypoints;
    }

    public Waypoint[] getPoints(double x, double y) {
      Waypoint[] waypoints = new Waypoint[points.length];
      for (int i = 0; i < waypoints.length; i++) {
        waypoints[i] = new Waypoint(points[i].x, points[i].y, points[i].angle);
      }
      if (first) {
        for (int i = 1; i < waypoints.length; i++) {
          waypoints[i].x += x;
          waypoints[i].y += y;
        }
      } else {
        for (Waypoint waypoint : waypoints) {
          waypoint.x += x;
          waypoint.y += y;
        }
      }
      return waypoints;
    }

    public Waypoint[] getPoints() {
      return points;
    }

    public boolean isBothSides() {
      return bothSides;
    }

    public Waypoint[] getLeftPoints() {
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

}
