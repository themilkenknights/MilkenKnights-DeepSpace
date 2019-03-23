package frc.robot.lib.util;

import frc.robot.Constants;
import frc.robot.lib.math.trajectory.Path;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.modifiers.TankModifier;
import java.io.File;
import java.io.IOException;

public class DeserializePath {

  /*
  Read CSV Files into memory and create left and right sides
   */
  public static Path getPathFromFile(String name) {
    try {
      String filePath = "/home/lvuser/deploy/" + name + ".csv";
      Trajectory traj = Pathfinder.readFromCSV(new File(filePath));
      TankModifier modifier = new TankModifier(traj).modify(Constants.DRIVE.kEffectiveDriveWheelTrackWidthInches);
      Trajectory left = modifier.getLeftTrajectory();
      Trajectory right = modifier.getRightTrajectory();
      return new Path(name, new Path.Pair(left, right));
    } catch (IOException ex) {
      Logger.logMarker("Crashed Trying to Deserialize Paths");
      Logger.logThrowableCrash(ex);
      return null;
    }
  }
}
