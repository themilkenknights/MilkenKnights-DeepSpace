import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class PathGenerator {

  public static final HashMap<String, Path> robotPaths = new HashMap<>();

  public static final Trajectory.Config defaultConfig = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_FAST,
      0.01, 100, 95, 1500);

  static {
  /*  robotPaths.put("CS-1", new Path(new Waypoint[] {
        new Waypoint(68 ,-47 ,Pathfinder.d2r(0)),
        new Waypoint(236 + 4 ,-102 - 12 ,Pathfinder.d2r(0)),
        new Waypoint(256 + 4 ,-78 - 12 ,Pathfinder.d2r(90)),
    }, veryFastConfig));

    robotPaths.put("CS-2", new Path(new Waypoint[] {
        new Waypoint(261 ,-46 ,Pathfinder.d2r(90)),
        new Waypoint(261 ,-48 ,Pathfinder.d2r(90)),
        new Waypoint(271 ,-70 ,Pathfinder.d2r(135)),
    }, veryFastConfig));

    robotPaths.put("CS-3", new Path(new Waypoint[] {
        new Waypoint(271 ,-70 ,Pathfinder.d2r(135)),
        new Waypoint(243 ,-60 - 14 ,Pathfinder.d2r(180)),
        new Waypoint(23 ,-135 - 14,Pathfinder.d2r(180)),
    }, veryFastConfig));

    robotPaths.put("CS-4", new Path(new Waypoint[] {
        new Waypoint(20 ,-136 ,Pathfinder.d2r(10)),
        new Waypoint(260 ,-59 - 30 ,Pathfinder.d2r(5)),
        new Waypoint(285 ,-79 - 30 ,Pathfinder.d2r(95)),
    }, veryFastConfig)); */

    robotPaths.put("CS-1", new Path(new Waypoint[] {
        new Waypoint(204, -12, Pathfinder.d2r(0)),
        new Waypoint(180, 19, Pathfinder.d2r(90)),
    }, defaultConfig));

    robotPaths.put("CS-2", new Path(new Waypoint[] {
        new Waypoint(180, 19, Pathfinder.d2r(90)),
        new Waypoint(70, -136, Pathfinder.d2r(0)),
        new Waypoint(32, -136, Pathfinder.d2r(0)),
    }, defaultConfig));

    robotPaths.put("CS-3", new Path(new Waypoint[] {
        new Waypoint(20, -136, Pathfinder.d2r(0)),
        new Waypoint(234, -62, Pathfinder.d2r(5)),
        new Waypoint(264, -86, Pathfinder.d2r(95)),
    }, defaultConfig));
  }

  public static void main(String[] args) {
    double tiL = 0;
    double tiR = 0;
    for (Map.Entry<String, Path> container : robotPaths.entrySet()) {
      if (container.getValue().isBothSides()) {
        Path traj = container.getValue();

        File leftPathFile = new File("src/main/deploy/" + container.getKey() + "L.csv").getAbsoluteFile();
        File rightPathFile = new File("src/main/deploy/" + container.getKey() + "R.csv").getAbsoluteFile();

        Trajectory leftTraj = Pathfinder.generate(traj.getLeftPoints(), traj.getConfig());
        Trajectory rightTraj = Pathfinder.generate(traj.getPoints(), traj.getConfig());

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
