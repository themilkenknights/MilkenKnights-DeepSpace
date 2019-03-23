import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class PathGenerator {

  public static final HashMap<String, Path> robotPaths = new HashMap<>();
/*  public static final Trajectory.Config fastConfig = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH,
      0.01, 130, 107, 1007);*/

  public static final Trajectory.Config slowerConfig = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH,
      0.01, 120, 105, 1007);

  public static final Trajectory.Config fastConfig = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_HIGH,
      0.01, 50, 25, 250);

  static {
    robotPaths.put("CS-1", new Path(new Waypoint[] {
        new Waypoint(68 ,0 ,Pathfinder.d2r(0)),
        new Waypoint(175 ,-11 ,Pathfinder.d2r(0)),
    }, slowerConfig));

    robotPaths.put("CS-2", new Path(new Waypoint[] {
        new Waypoint(203 ,-11 ,Pathfinder.d2r(0)),
        new Waypoint(60 ,-136 ,Pathfinder.d2r(0)),
    }, slowerConfig));

    robotPaths.put("CS-3", new Path(new Waypoint[] {
        new Waypoint(20 ,-136 ,Pathfinder.d2r(0)),
        new Waypoint(261 ,-90 ,Pathfinder.d2r(0)),
    }, slowerConfig));
  }

  public static void main(String[] args) {
    double tiL = 0;
    double tiR = 0;
    for (Map.Entry<String, Path> container : robotPaths.entrySet()) {
      if (container.getValue().isBothSides()) {
        Path traj = container.getValue();

        File leftPathFile = new File("src/main/deploy/" + container.getKey() + "L.csv").getAbsoluteFile();
        File rightPathFile = new File("src/main/deploy/" + container.getKey() + "R.csv").getAbsoluteFile();

        Trajectory leftTraj = Pathfinder.generate(traj.getPoints(), traj.getConfig());
        Trajectory rightTraj = Pathfinder.generate(traj.getRightPoints(), traj.getConfig());

        Pathfinder.writeToCSV(leftPathFile, leftTraj);
        Pathfinder.writeToCSV(rightPathFile, rightTraj);

        System.out.println("Path: " + container.getKey() + " Time: " + leftTraj.length() * 0.01 + " Sec");
        if (container.getKey().charAt(0) == 'C') {
          tiL += leftTraj.length() * 0.01;
          tiR += rightTraj.length() * 0.01;
        }
      } else {
        File pathFile = new File("paths/" + container.getKey() + ".csv").getAbsoluteFile();
        Trajectory trajectory = Pathfinder.generate(container.getValue().getPoints(), container.getValue().getConfig());
        Pathfinder.writeToCSV(pathFile, trajectory);
        System.out.println("Path: " + container.getKey() + " Time: " + trajectory.length() * 0.005 + " Sec");
      }
    }
    System.out.println("Left: " + tiL + " Right: " + tiR);
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
}
