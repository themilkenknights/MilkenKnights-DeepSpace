package frc.robot.planners;

import frc.robot.Constants;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.lib.geometry.Translation2d;
import frc.robot.lib.geometry.Twist2d;
import frc.robot.lib.trajectory.TimedView;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.paths.DriveMotionPlanner;
import frc.robot.paths.Kinematics;
import org.junit.jupiter.api.Test;

import java.util.Arrays;

public class DriveMotionPlannerTest {
		@Test public void testFollowerReachesGoal() {
				final DriveMotionPlanner motion_planner = new DriveMotionPlanner();
				motion_planner.setFollowerType(DriveMotionPlanner.FollowerType.NONLINEAR_FEEDBACK);
				motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false, Arrays
						.asList(new Pose2d(new Translation2d(0.0, 0.0), Rotation2d.identity()), new Pose2d(new Translation2d(120.0, -36.0), Rotation2d.identity()),
								new Pose2d(new Translation2d(240.0, -36.0), Rotation2d.identity())), null, 120.0, 120.0, 10.0))));
				final double dt = 0.01;
				double t = 0.0;
				Pose2d initial_error = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(3.5));
				Pose2d pose = motion_planner.setpoint().state().getPose().transformBy(initial_error);
				while (!motion_planner.isDone()) {
						DriveMotionPlanner.Output output = motion_planner.update(t, pose);
						Twist2d delta = Kinematics.forwardKinematics(output.left_velocity * dt * Constants.WHEEL_DIAMETER / 2.0, output.right_velocity * dt * Constants.WHEEL_DIAMETER / 2.0);
						// Add some systemic error.
						delta = new Twist2d(delta.dx * 1.0, delta.dy * 1.0, delta.dtheta * 1.05);
						pose = pose.transformBy(Pose2d.exp(delta));
						t += dt;
						System.out.println(motion_planner.setpoint().toCSV() + "," + pose.toCSV());
				}
				System.out.println(pose);
		}

		@Test public void testVoltages() {
				final DriveMotionPlanner motion_planner = new DriveMotionPlanner();
				motion_planner.setTrajectory(new TrajectoryIterator<>(new TimedView<>(motion_planner.generateTrajectory(false, Arrays
						.asList(Pose2d.identity(), Pose2d.fromTranslation(new Translation2d(48.0, 0.0)), new Pose2d(new Translation2d(96.0, 48.0), Rotation2d.fromDegrees(90.0)),
								new Pose2d(new Translation2d(96.0, 96.0), Rotation2d.fromDegrees(90.0))), null, 48.0, 48.0, 10.0))));
				double t = 0.0;
				Pose2d pose = motion_planner.setpoint().state().getPose().transformBy(new Pose2d(Translation2d.identity(), Rotation2d.fromDegrees(180.0)));
				while (!motion_planner.isDone()) {
						motion_planner.update(t, pose);
						pose = motion_planner.mSetpoint.state().getPose();
						System.out.println(t + "," + motion_planner.toCSV());
						t += 0.01;
				}
		}
}
