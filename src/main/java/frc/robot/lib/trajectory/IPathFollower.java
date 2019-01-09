package frc.robot.lib.trajectory;

import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Twist2d;

public interface IPathFollower {
    public Twist2d steer(Pose2d current_pose);

    public boolean isDone();
}
