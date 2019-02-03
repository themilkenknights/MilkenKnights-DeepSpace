package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.DRIVE;
import frc.robot.Robot;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.vision.LimelightTarget;
import frc.robot.lib.vision.VisionState;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.HatchArm.HatchMechanismState;

public class Superstructure extends Subsystem {

	private static Drive mDrive = Drive.getInstance();
	private static HatchArm mHatch = HatchArm.getInstance();
	private static CargoArm mCargo = CargoArm.getInstance();
	private PowerDistributionPanel mPDP;
	private Compressor mCompressor;
	private RobotState mRobotState = RobotState.TELEOP_DRIVE;
	private boolean hasHatch = false;
	private VisionState mLastVisionState = VisionState.EMPTY;

	private Superstructure() {
		mPDP = new PowerDistributionPanel(Constants.CAN.kPowerDistributionPanelID);
		mCompressor = new Compressor(CAN.kPneumaticsControlModuleID);
	}

	public static Superstructure getInstance() {
		return InstanceHolder.mInstance;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("Robot State", Robot.mMatchState.toString());
		SmartDashboard.putNumber("Compressor Current", mCompressor.getCompressorCurrent());
		if (mPDP.getVoltage() < 11.5) {
			DriverStation.reportWarning("Low Battery Voltage", false);
		}
	}

	@Override
	public void zero(double timestamp) {
	}

	@Override
	public void onLoop(double timestamp) {

		switch (mRobotState) {

			case TELEOP_DRIVE:
				break;
			case VISION_INTAKE_STATION:
			case VISION_PLACING:
				if (!mHatch.hatchOnArmLimit() && !hasHatch) {
					LimelightTarget target = Vision.getInstance().getAverageTarget();
					double mDist = target.getDistance();
					if (mDist > 15.0 && target.isValidTarget()) {
						double mSteer = DRIVE.kVisionTurnP * target.getXOffset();
						DriveSignal mSig = mDrive
								.updateMotionMagicDeltaSetpoint(new DriveSignal(mDist, mDist, NeutralMode.Coast), new DriveSignal(mSteer, -mSteer));
						mLastVisionState = new VisionState(mSig, target, mDrive.getFusedNormalizedHeading());
					} else {
						double mSteer = DRIVE.kVisionTurnP * (mLastVisionState.getTarget().getXOffset() - mDrive.getFusedNormalizedHeading());
						mDrive.updateMotionMagicPositionSetpoint(mLastVisionState.getDriveSignal(), new DriveSignal(mSteer, -mSteer));
						mHatch.setHatchMechanismState(
								mRobotState == RobotState.VISION_INTAKE_STATION ? HatchMechanismState.STATION_INTAKE : HatchMechanismState.PLACING);
					}
				} else if (HatchArm.getInstance().hatchOnArmLimit() && !hasHatch) {
					hasHatch = true;
					mDrive.updateMotionMagicDeltaSetpoint(new DriveSignal(-20.0, -20.0, NeutralMode.Brake), DriveSignal.BRAKE);
					if (mRobotState == RobotState.VISION_INTAKE_STATION) {
						HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.STOWED);
					}
				} else if (hasHatch && mDrive.isMotionMagicFinished() && !mDrive.isTurnDone()) {
					mDrive.updateTurnToHeading(180.0);
				} else if (mDrive.isTurnDone()) {
					setRobotState(RobotState.TELEOP_DRIVE);
				} else {
					Logger.logCriticalError("Unexpected Vision Intake State");
				}
				break;
			default:
				Logger.logCriticalError("Unexpected robot state: " + mRobotState);
				break;

		}

	}

	public RobotState getRobotState() {
		return mRobotState;
	}

	public void setRobotState(RobotState state) {
		resetActionVariables();
		Logger.logMarker("Switching to Robot State:" + mRobotState);
		mRobotState = state;
		switch (state) {
			case TELEOP_DRIVE:
				mHatch.setHatchMechanismState(HatchMechanismState.STOWED);
				mCargo.setArmState(CargoArmState.STOW);
				break;
			case VISION_INTAKE_STATION:
			case VISION_PLACING:
				mRobotState = Vision.getInstance().getAverageTarget().isValidTarget() ? mRobotState : RobotState.TELEOP_DRIVE;
				break;
			default:
				Logger.logCriticalError("Unexpected robot state: " + mRobotState);
				break;
		}
	}

	@Override
	public void onStop(double timestamp) {
	}

	@Override
	public boolean checkSystem() {
		return mCompressor.getCompressorCurrent() > 0.0 && mPDP.getTotalCurrent() > 0.0;
	}

	private void resetActionVariables() {
		hasHatch = false;
		mLastVisionState = VisionState.EMPTY;
	}

	public enum RobotState {
		PATH_FOLLOWING, TELEOP_DRIVE, VISION_INTAKE_STATION, VISION_PLACING, VISION_CARGO_INTAKE, VISION_CARGO_OUTTAKE
	}

	private static class InstanceHolder {

		private static final Superstructure mInstance = new Superstructure();
	}
}
