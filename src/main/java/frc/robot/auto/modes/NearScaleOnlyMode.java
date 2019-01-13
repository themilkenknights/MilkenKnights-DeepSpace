package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveTrajectory;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.paths.TrajectoryGenerator.TrajectorySet.MirroredTrajectory;

public class NearScaleOnlyMode extends AutoModeBase {
		private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
		private final boolean mStartedLeft;
		private DriveTrajectory mSideStartToNearScale;

		public NearScaleOnlyMode(boolean robotStartedOnLeft) {
				mStartedLeft = robotStartedOnLeft;
				MirroredTrajectory a = mTrajectoryGenerator.getTrajectorySet().sideStartToNearScale;
				mSideStartToNearScale = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().sideStartToNearScale.get(mStartedLeft), true);
		}

		@Override protected void routine() throws AutoModeEndedException {
				runAction(mSideStartToNearScale);
		}
}
