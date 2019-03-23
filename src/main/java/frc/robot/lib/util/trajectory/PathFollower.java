package frc.robot.lib.util.trajectory;

import frc.robot.Constants;
import frc.robot.lib.util.TrajectoryStatus;

public class PathFollower {
  protected TrajectoryFollower lFollower;
  protected TrajectoryFollower rFollower;
  private Path path;

  public PathFollower(Path mPath, double distTol, double angTol) {
    path = mPath;
    lFollower = new TrajectoryFollower(mPath.getLeftWheelTrajectory());
    lFollower.configure(Constants.DRIVE.DRIVE_FOLLOWER_P, Constants.DRIVE.DRIVE_FOLLOWER_A, Constants.DRIVE.DRIVE_FOLLOWER_ANG, distTol, angTol);
    rFollower = new TrajectoryFollower(mPath.getRightWheelTrajectory());
    rFollower.configure(Constants.DRIVE.DRIVE_FOLLOWER_P, Constants.DRIVE.DRIVE_FOLLOWER_A, -Constants.DRIVE.DRIVE_FOLLOWER_ANG, distTol, angTol);
  }

  public TrajectoryStatus getLeftVelocity(double dist, double vel, double angle) {
    return lFollower.calculate(dist, vel, angle);
  }

  public TrajectoryStatus getRightVelocity(double dist, double vel, double angle) {
    return rFollower.calculate(dist, vel, angle);
  }

  public boolean getFinished() {
    return lFollower.isFinishedTrajectory() && rFollower.isFinishedTrajectory();
  }

  public double getTimeLeft() {
    return (lFollower.timeLeft() + rFollower.timeLeft()) / 2.0;
  }

  public double getEndHeading() {
    return path.getEndHeading();
  }
}
