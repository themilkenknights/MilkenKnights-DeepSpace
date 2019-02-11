package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AutoChooser;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.DRIVE;
import frc.robot.Robot;
import frc.robot.auto.AutoModeExecutor;
import frc.robot.auto.modes.PathTrackTarget;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTime;
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
	private boolean hasHatch, inPosition, startedTurn = false;
	private VisionState mLastVisionState = VisionState.EMPTY;
	private double mGoalTurnAngle = 0;
	private MkTime mRandomTimer = new MkTime();
	private static AutoModeExecutor mAutoModeExecuter = null;

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
	public void onLoop(double timestamp) {

		switch (mRobotState) {
			case TELEOP_DRIVE:
				break;
			case VISION_INTAKE_STATION:
			case VISION_PLACING:
				//updateVisionHatch();
				break;
			case VISION_CARGO_OUTTAKE:
				updateVisionCargo();
				break;
			default:
				Logger.logErrorWithTrace("Unexpected robot state: " + mRobotState);
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
				AutoChooser.disableAuto();
				mDrive.setOpenLoop(DriveSignal.BRAKE);
				mHatch.setHatchMechanismState(HatchMechanismState.STOWED);
				mCargo.setArmState(CargoArmState.STOW);
				break;
			case VISION_INTAKE_STATION:
			case VISION_PLACING:
				mDrive.setOpenLoop(DriveSignal.BRAKE);
				//mRobotState = Vision.getInstance().getAverageTarget().isValidTarget() ? mRobotState : RobotState.TELEOP_DRIVE;
				startVisionHatch();
				break;
			case VISION_CARGO_OUTTAKE:
				mRobotState = Vision.getInstance().getAverageTarget().isValidTarget() ? mRobotState : RobotState.TELEOP_DRIVE;
			default:
				Logger.logErrorWithTrace("Unexpected robot state: " + mRobotState);
				break;
		}
	}

	private void startVisionHatch() {
		AutoChooser.startAuto(new PathTrackTarget(Vision.getInstance().getAverageTarget().getDeltaPose()));
		mHatch.setHatchMechanismState(mRobotState == RobotState.VISION_INTAKE_STATION ? HatchMechanismState.STATION_INTAKE : HatchMechanismState.PLACING);
	}

	private synchronized void updateVisionHatch() {
		if (!mHatch.isHatchLimitTriggered() && !hasHatch) {
			LimelightTarget target = Vision.getInstance().getAverageTarget();
			double mDist = target.getDistance();
			if (mDist > 20.0 && target.isValidTarget()) {
				double mSteer = DRIVE.kVisionTurnP * target.getXOffset();
				DriveSignal mSig = mDrive.updateMotionMagicDeltaSetpoint(new DriveSignal(mDist, mDist, NeutralMode.Coast), new DriveSignal(mSteer, -mSteer));
				mLastVisionState = new VisionState(mSig, target, mDrive.getFusedHeading());
			} else {
				double mSteer = DRIVE.kVisionTurnP * (mLastVisionState.getTarget().getXOffset() - mDrive.getYaw());
				mDrive.updateMotionMagicPositionSetpoint(mLastVisionState.getDriveSignal(), new DriveSignal(mSteer, -mSteer));
				mHatch.setHatchMechanismState(HatchMechanismState.VISION_CONTROL);
			}
		} else if (HatchArm.getInstance().isHatchLimitTriggered() && !hasHatch) {
			hasHatch = true;
			mDrive.updateMotionMagicDeltaSetpoint(new DriveSignal(-20.0, -20.0, NeutralMode.Brake), DriveSignal.BRAKE);
			if (mRobotState == RobotState.VISION_INTAKE_STATION) {
				HatchArm.getInstance().setHatchMechanismState(HatchMechanismState.STOWED);
			}
		} else if (hasHatch && mDrive.isMotionMagicFinished() && !startedTurn) {
			startedTurn = true;
			mGoalTurnAngle = MkMath.normalAbsoluteAngleDegrees(Drive.getInstance().getFusedHeading() + 180.0);
			mDrive.updateTurnToHeading(mGoalTurnAngle);
		} else if (!mDrive.isTurnDone() && startedTurn) {
			mDrive.updateTurnToHeading(mGoalTurnAngle);
		} else if (mDrive.isTurnDone()) {
			setRobotState(RobotState.TELEOP_DRIVE);
		} else {
			Logger.logErrorWithTrace("Unexpected Vision Intake State");
		}
	}

	private synchronized void updateVisionCargo() {
		if (!inPosition) {
			LimelightTarget target = Vision.getInstance().getAverageTarget();
			double mDist = target.getDistance();
			if (mDist > 15.0 && target.isValidTarget()) {
				double mSteer = DRIVE.kVisionTurnP * target.getXOffset();
				DriveSignal mSig = mDrive.updateMotionMagicDeltaSetpoint(new DriveSignal(mDist, mDist, NeutralMode.Coast), new DriveSignal(mSteer, -mSteer));
				mLastVisionState = new VisionState(mSig, target, mDrive.getFusedHeading());
			} else if (!Drive.getInstance().isMotionMagicFinished()) {
				double mSteer = DRIVE.kVisionTurnP * (mLastVisionState.getTarget().getXOffset() - mDrive.getYaw());
				mDrive.updateMotionMagicPositionSetpoint(mLastVisionState.getDriveSignal(), new DriveSignal(mSteer, -mSteer));
			} else {
				inPosition = true;
			}
		} else if (!startedTurn) {
			startedTurn = true;
			mGoalTurnAngle = MkMath.normalAbsoluteAngleDegrees(Drive.getInstance().getFusedHeading() + 180.0);
			mDrive.updateTurnToHeading(mGoalTurnAngle);
			mCargo.setArmState(CargoArmState.PLACE);
		} else if (!mDrive.isTurnDone() && startedTurn) {
			mDrive.updateTurnToHeading(mGoalTurnAngle);
		} else if (mDrive.isTurnDone() && !mRandomTimer.hasBeenSet()) {
			mRandomTimer.start(0.75);
			mCargo.setIntakeRollers(CARGO_ARM.INTAKE_OUT_ROLLER_SPEED);
		} else if (!mRandomTimer.isDone()) {
			mCargo.setIntakeRollers(CARGO_ARM.INTAKE_OUT_ROLLER_SPEED);
		} else {
			setRobotState(RobotState.TELEOP_DRIVE);
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
		mGoalTurnAngle = 0.0;
	}

	public enum RobotState {
		PATH_FOLLOWING, TELEOP_DRIVE, VISION_INTAKE_STATION, VISION_PLACING, VISION_CARGO_INTAKE, VISION_CARGO_OUTTAKE
	}

	private static class InstanceHolder {

		private static final Superstructure mInstance = new Superstructure();
	}
}
