package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.geometry.Pose2dWithCurvature;
import frc.robot.lib.trajectory.TimedView;
import frc.robot.lib.trajectory.Trajectory;
import frc.robot.lib.trajectory.TrajectoryIterator;
import frc.robot.lib.trajectory.timing.TimedState;
import frc.robot.lib.util.DriveSignal;
import frc.robot.paths.RobotState;
import frc.robot.subsystems.Drive;

public class DriveTrajectory implements Action {

	private static final Drive mDrive = Drive.getInstance();
	private static final RobotState mRobotState = RobotState.getInstance();

	private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
	private final boolean mResetPose;

	public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
		this(trajectory, false);
	}


	public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean resetPose) {
		mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
		mResetPose = resetPose;
	}

	@Override
	public boolean isFinished() {
		if (mDrive.isDoneWithTrajectory()) {
			Drive.getInstance().setOpenLoop(new DriveSignal(0, 0));
			System.out.println("Trajectory finished");
			return true;
		}
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
		System.out.println("Starting trajectory! (length=" + mTrajectory.getRemainingProgress() + ")");
		if (mResetPose) {
			mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
		}
		mDrive.setTrajectory(mTrajectory);
	}
}

