import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import java.io.File;
import java.util.HashMap;
import java.util.Map;

public class PathGenerator {

  public static final HashMap<String, PathContainer> robotPaths = new HashMap<>();

  public static final Trajectory.Config defaultConfig = new Trajectory.Config(
      Trajectory.FitMethod.HERMITE_QUINTIC, Trajectory.Config.SAMPLES_FAST,
      0.01, 105, 80, 1500);

  static {
    robotPaths.put("CS-1", new PathContainer(new Waypoint[] {
        new Waypoint(204, -12, Pathfinder.d2r(0)),
        new Waypoint(169, 24, Pathfinder.d2r(90)),
    }, defaultConfig));

    robotPaths.put("CS-2", new PathContainer(new Waypoint[] {
        new Waypoint(169, 24, Pathfinder.d2r(90)),
        new Waypoint(70, -136, Pathfinder.d2r(0)),
        new Waypoint(35, -136, Pathfinder.d2r(0)),
    }, defaultConfig));

    robotPaths.put("CS-3", new PathContainer(new Waypoint[] {
        new Waypoint(20, -136, Pathfinder.d2r(4)),
        new Waypoint(234, -70, Pathfinder.d2r(5)),
        new Waypoint(264, -94, Pathfinder.d2r(95)),
    }, defaultConfig));
  }

  public static void main(String[] args) {
    double tiL = 0;
    double tiR = 0;
    for (Map.Entry<String, PathContainer> container : robotPaths.entrySet()) {
      PathContainer traj = container.getValue();

      File leftPathFile = new File("src/main/deploy/" + container.getKey() + "L.csv").getAbsoluteFile();
      File rightPathFile = new File("src/main/deploy/" + container.getKey() + "R.csv").getAbsoluteFile();

      Trajectory leftTraj = Pathfinder.generate(traj.getPoints(), traj.getConfig());
      Trajectory rightTraj = Pathfinder.generate(traj.getRightPoints(), traj.getConfig());

      System.out.println("Path: " + container.getKey() + " Time: " + leftTraj.length() * 0.01 + " Sec");
      if (container.getKey().charAt(0) == 'C') {
        tiL += leftTraj.length() * 0.01;
        tiR += rightTraj.length() * 0.01;
      }
    }
    System.out.println("Left: " + tiL + " Right: " + tiR);
  }

  static class PathContainer {

    Waypoint[] points;
    Trajectory.Config config;

    public PathContainer(Waypoint[] points, Trajectory.Config config) {
      this.points = points;
      this.config = config;
    }

    public Waypoint[] getPoints() {
      return points;
    }

    public Waypoint[] getRightPoints() {
      Waypoint[] waypoints = new Waypoint[points.length];
      for (int i = 0; i < waypoints.length; i++) {
        waypoints[i] = new Waypoint(points[i].x, points[i].y, points[i].angle);
      }
      for (Waypoint waypoint : waypoints) {
        waypoint.y *= -1.0;
        waypoint.angle *= -1.0;
      }
      return waypoints;
    }

    public Trajectory.Config getConfig() {
      return config;
    }
  }
}
