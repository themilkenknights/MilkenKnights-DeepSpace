package frc.robot.auto.actions;

import frc.robot.Constants.DRIVE;
import frc.robot.lib.util.trajectory.Path;
import frc.robot.subsystems.Drive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class DrivePathVision implements Action {

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void update() {

  }

  @Override
  public void done() {

  }

  @Override
  public void start() {
    Trajectory.Config fastConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_QUINTIC, Config.SAMPLES_FAST,
        0.02, 135, 80, 500);
    Waypoint[] drivePath = new Waypoint[]{
        new Waypoint(0, 0, Pathfinder.d2r(0)),
        new Waypoint(35.5, 8.323, Pathfinder.d2r(23.343))};

    Trajectory trajectory = Pathfinder.generate(drivePath, fastConfig);
    TankModifier modifier = new TankModifier(trajectory).modify(DRIVE.kEffectiveDriveWheelTrackWidthInches);
    Trajectory left = modifier.getLeftTrajectory();
    Trajectory right = modifier.getRightTrajectory();
    Drive.getInstance().setDrivePath(new Path("Vision path", new Path.Pair(left, right)), 3.0, 1.0);
  }

}
