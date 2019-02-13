package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.DRIVE;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.MkTime;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.lib.vision.VisionState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Vision;

public class MotionMagicVision implements Action {

	private MkTime expirationTimer;
	private VisionState mLastVisionState = VisionState.EMPTY;
	private boolean useLimit;

	public MotionMagicVision(boolean useLimit) {
		expirationTimer = new MkTime();
		this.useLimit = useLimit;
	}


	@Override
	public boolean isFinished() {
		return expirationTimer.isDone() || (useLimit ? HatchArm.getInstance().isHatchLimitTriggered() : Drive.getInstance().isMotionMagicFinished());
	}

	@Override
	public void update() {
		LimelightTarget target = Vision.getInstance().getAverageTarget();
		double mDist = target.getDistance();
		if (mDist > 20.0 && target.isValidTarget()) {
			double mSteer = DRIVE.kVisionTurnP * target.getXOffset();
			DriveSignal mSig = Drive.getInstance().setMotionMagicDeltaSetpoint(new DriveSignal(mDist, mDist, NeutralMode.Coast), new DriveSignal(mSteer, -mSteer));
			mLastVisionState = new VisionState(mSig, target, Drive.getInstance().getFusedHeading());
		} else {
			double mSteer = DRIVE.kVisionTurnP * (mLastVisionState.getTarget().getXOffset() - Drive.getInstance().getYaw());
			Drive.getInstance().updateMotionMagicPositionSetpoint(mLastVisionState.getDriveSignal(), new DriveSignal(mSteer, -mSteer));
		}
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		expirationTimer.start(5.0);
	}
}
