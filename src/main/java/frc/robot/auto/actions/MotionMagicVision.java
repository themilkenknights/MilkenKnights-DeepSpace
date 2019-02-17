package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Timer;
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
	private double lastTime = 0.0;
	private double targetYaw = 0.0;
	private double initYaw = 0.0;

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
			double vel = (mLastVisionState.getTarget().getXOffset() - target.getXOffset())/(Timer.getFPGATimestamp() - lastTime);
			double mSteer = DRIVE.kVisionTurnP * -(Drive.getInstance().getYaw() - (targetYaw + initYaw)) + vel * DRIVE.kVisionTurnP * 8;
			DriveSignal mSig = Drive.getInstance().setMotionMagicDeltaSetpoint(new DriveSignal(mDist, mDist, NeutralMode.Coast), new DriveSignal(mSteer, -mSteer));
			mLastVisionState = new VisionState(mSig, target, Drive.getInstance().getFusedHeading());
		} else {
			double mSteer = DRIVE.kVisionTurnP * 0 * (mLastVisionState.getTarget().getXOffset() - /*Math.abs(mLastVisionState.getYaw() - Drive.getInstance().getYaw())*/ 0);
			Drive.getInstance().updateMotionMagicPositionSetpoint(mLastVisionState.getDriveSignal(), new DriveSignal(mSteer, -mSteer));
		}
	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		expirationTimer.start(5.0);
		lastTime = Timer.getFPGATimestamp();
		targetYaw = Vision.getInstance().getAverageTarget().getXOffset();
		initYaw = Drive.getInstance().getYaw();
	}
}