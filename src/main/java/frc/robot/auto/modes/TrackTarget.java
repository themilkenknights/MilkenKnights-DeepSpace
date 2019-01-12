package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveTrajectory;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.geometry.Rotation2d;
import frc.robot.paths.TrajectoryGenerator;

public class TrackTarget extends AutoModeBase {

	private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
	private DriveTrajectory mTraj;

	public TrackTarget(Pose2d endPose) {
		mTrajectoryGenerator.generateVisionTrajectories(endPose);
		mTraj = new DriveTrajectory(mTrajectoryGenerator.getVisionTrajectorySet().visionTraj, true);
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(mTraj);
	}
}
