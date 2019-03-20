package frc.robot.lib.util;

import java.io.File;
import frc.robot.Constants;
import frc.robot.lib.util.trajectory.Path;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.modifiers.TankModifier;

public class DeserializePath {
  /*
   * Read CSV Files into memory and create left and right sides
   */
  public static Path getPathFromFile(String name) {
    try {
      String filePath = Constants.AUTO.pathPath + name + ".csv";
      Trajectory traj = Pathfinder.readFromCSV(new File(filePath));
      TankModifier modifier = new TankModifier(traj).modify(Constants.DRIVE.PATH_WHEELBASE);
      Trajectory left = modifier.getLeftTrajectory();
      Trajectory right = modifier.getRightTrajectory();
      return new Path(name, new Path.Pair(left, right));
    } catch (Throwable t) {
      Logger.logError("Crashed Trying to Deserialize Paths");
      Logger.logThrowableCrash(t);
      throw t;
    }
  }
}
