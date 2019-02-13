package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.DRIVE;
import frc.robot.Constants.VISION;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.InterpolatingDouble;
import frc.robot.lib.util.MkTime;
import frc.robot.lib.vision.VisionState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Vision;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

public class MotionMagicPixy implements Action {

	private MkTime expirationTimer;
	private VisionState mLastVisionState = VisionState.EMPTY;

	public MotionMagicPixy() {
		expirationTimer = new MkTime();
	}


	@Override
	public boolean isFinished() {
		return expirationTimer.isDone() || Vision.getInstance().mPixy.isCargoIntaked();
	}

	@Override
	public void update() {
		Block target = Vision.getInstance().mPixy.getLastBlock();
		double mDist = VISION.kPixyAreaToDistVisionMap.getInterpolated(new InterpolatingDouble((double) (target.getHeight() * target.getWidth()))).value;
		if (mDist > 5.0 && mDist < 60) {
			double mSteer = DRIVE.kVisionTurnP * target.getX();
			DriveSignal mSig = Drive.getInstance().setMotionMagicDeltaSetpoint(new DriveSignal(mDist, mDist, NeutralMode.Coast), new DriveSignal(mSteer, -mSteer));
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
