package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DRIVE;
import frc.robot.lib.util.trajectory.Path;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class DrivePathVision implements Action {
  @Override
  public boolean isFinished() {
    return Drive.getInstance().isPathFinished();
  }

  @Override
  public void update() {
  }

  @Override
  public void done() {
    //HatchArm.getInstance().setHatchState(HatchArm.HatchState.PLACE);
    //Superstructure.getInstance().setRobotState(Superstructure.RobotState.TELEOP_DRIVE);
  }

  @Override
  public void start() {
    double now = Timer.getFPGATimestamp();
    Trajectory.Config fastConfig = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Config.SAMPLES_LOW, DRIVE.PATH_DT, 38, 30, 150);
    LimelightTarget currentTarget = Vision.getInstance().getLimelightTarget();
    double x = currentTarget.getCamTran()[2];
    double y = -currentTarget.getCamTran()[0];
    double yaw = currentTarget.getCamTran()[4];
    Waypoint[] drivePath = new Waypoint[] {new Waypoint(x, y, Pathfinder.d2r(yaw)), new Waypoint(0.0, 0.0, Pathfinder.d2r(0.0))};
    Trajectory trajectory = Pathfinder.generate(drivePath, fastConfig);
    TankModifier modifier = new TankModifier(trajectory).modify(DRIVE.kEffectiveDriveWheelTrackWidthInches);
    Trajectory left = modifier.getLeftTrajectory();
    Trajectory right = modifier.getRightTrajectory();
    Drive.getInstance().setDrivePath(new Path("Vision path", new Path.Pair(left, right)), 1.0, 1.0);
    double dt = Timer.getFPGATimestamp() - now;
    System.out.println(dt);
  }
}
