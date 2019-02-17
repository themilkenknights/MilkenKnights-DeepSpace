package frc.robot.auto.modes;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.DriveTrajectory;
import frc.robot.paths.TrajectoryGenerator;

public class TestPathMode extends AutoModeBase {

	private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
	private final boolean mStartedLeft;
	private DriveTrajectory mSideStartToNearScale;

	public TestPathMode(boolean robotStartedOnLeft) {
		mStartedLeft = robotStartedOnLeft;
		mSideStartToNearScale = new DriveTrajectory(
				mTrajectoryGenerator.getTrajectorySet().testPath.get(mStartedLeft), true, false);
	}

	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(mSideStartToNearScale);
	}
}
