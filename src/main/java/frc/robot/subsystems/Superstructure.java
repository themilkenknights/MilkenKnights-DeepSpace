package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.AutoChooser;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.PNUEMATICS;
import frc.robot.Constants.SUPERSTRUCTURE;
import frc.robot.Robot;
import frc.robot.auto.modes.CargoVisionIntake;
import frc.robot.auto.modes.SimpleCargoOuttake;
import frc.robot.auto.modes.SimpleHatchVision;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.HatchArm.HatchMechanismState;

public class Superstructure extends Subsystem {

	private static Drive mDrive = Drive.getInstance();
	private static HatchArm mHatch = HatchArm.getInstance();
	private static CargoArm mCargo = CargoArm.getInstance();
	private PowerDistributionPanel mPDP;
	private Compressor mCompressor;
	private RobotState mRobotState = RobotState.TELEOP_DRIVE;
	private Solenoid mFrontClimbSolenoid, mRearClimbSolenoid;
	private ClimbState mRearClimbState = ClimbState.RETRACTED;
	private ClimbState mFrontClimbState = ClimbState.RETRACTED;
	private ShuffleboardTab mStructureTab;
	private NetworkTableEntry mMatchState, mRobotStateEntry, mCompressorCurrent, mFrontClimb, mRearClimb;

	private Superstructure() {
		mStructureTab = Shuffleboard.getTab("Superstructure");
		mMatchState = mStructureTab.add("Match State", "").getEntry();
		mRobotStateEntry = mStructureTab.add("Robot State", "").getEntry();
		mCompressorCurrent = mStructureTab.add("Compressor Current", 0.0).getEntry();
		mFrontClimb = mStructureTab.add("Front Climb", "").getEntry();
		mRearClimb = mStructureTab.add("Rear Climb", "").getEntry();

		mFrontClimbSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, PNUEMATICS.kFrontClimbSolenoidChannel);
		mRearClimbSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, PNUEMATICS.kRearClimbSolenoidChannel);

		mPDP = new PowerDistributionPanel(Constants.CAN.kPowerDistributionPanelID);
		LiveWindow.disableTelemetry(mPDP);
		mCompressor = new Compressor(CAN.kPneumaticsControlModuleID);
	}

	public static Superstructure getInstance() {
		return InstanceHolder.mInstance;
	}

	@Override
	public void outputTelemetry(double timestamp) {
		mMatchState.setString(Robot.mMatchState.toString());
		mRobotStateEntry.setString(mRobotState.toString());
		mCompressorCurrent.setDouble(mCompressor.getCompressorCurrent());
		mFrontClimb.setString(mFrontClimbState.toString());
		mRearClimb.setString(mRearClimbState.toString());
	}

	public ClimbState getFrontClimbState() {
		return mFrontClimbState;
	}

	public void setFrontClimbState(ClimbState state) {
		mFrontClimbState = state;
		mFrontClimbSolenoid.set(state.state);
	}

	public ClimbState getRearClimbState() {
		return mRearClimbState;
	}

	public void setRearClimbState(ClimbState state) {
		mRearClimbState = state;
		mRearClimbSolenoid.set(state.state);
	}

	@Override
	public void onMainLoop(double timestamp) {
		switch (mRobotState) {
			case PATH_FOLLOWING:
				break;
			case TELEOP_DRIVE:
				break;
			case VISION_INTAKE_STATION:
			case VISION_PLACING:
				//updateVisionHatch();
				break;
			case VISION_CARGO_INTAKE:
				break;
			case VISION_CARGO_OUTTAKE:
				break;
			default:
				Logger.logErrorWithTrace("Unexpected robot state: " + mRobotState);
				break;

		}
	}

	public void teleopInit(double timestamp) {
		setRobotState(RobotState.TELEOP_DRIVE);
	}

	public RobotState getRobotState() {
		return mRobotState;
	}

	public synchronized void setRobotState(RobotState state) {
		mRobotState = state;
		switch (state) {
			case TELEOP_DRIVE:
				AutoChooser.disableAuto();
				mDrive.setOpenLoop(DriveSignal.BRAKE);
				mHatch.setHatchMechanismState(HatchMechanismState.STOWED);
				break;
			case VISION_INTAKE_STATION:
			case VISION_PLACING:
				mDrive.setOpenLoop(DriveSignal.BRAKE);
				mRobotState = Vision.getInstance().getAverageTarget().isValidTarget() ? mRobotState : RobotState.TELEOP_DRIVE;
				startVisionHatch();
				break;
			case VISION_CARGO_OUTTAKE:
				mRobotState = Vision.getInstance().getAverageTarget().isValidTarget() ? mRobotState : RobotState.TELEOP_DRIVE;
				startVisionCargo();
			case VISION_CARGO_INTAKE:
				startVisionCargoIntake();
			case PATH_FOLLOWING:
				break;
			default:
				Logger.logErrorWithTrace("Unexpected robot state: " + mRobotState);
				break;
		}
		Logger.logMarker("Switching to Robot State:" + mRobotState);
	}

	private void startVisionHatch() {
		mHatch.setHatchMechanismState(mRobotState == RobotState.VISION_INTAKE_STATION ? HatchMechanismState.STATION_INTAKE : HatchMechanismState.PLACING);
		//AutoChooser.startAuto(new PathTrackTarget(Vision.getInstance().getAverageTarget().getDeltaPose()));
		AutoChooser.startAuto(new SimpleHatchVision());
	}

	private void startVisionCargo() {
		mHatch.setHatchMechanismState(HatchMechanismState.STOWED);
		AutoChooser.startAuto(new SimpleCargoOuttake());
	}


	private void startVisionCargoIntake() {

		AutoChooser.startAuto(new CargoVisionIntake());
	}

	@Override
	public void onStop(double timestamp) {
		setFrontClimbState(ClimbState.RETRACTED);
		setRearClimbState(ClimbState.RETRACTED);
	}

	@Override
	public boolean checkSystem() {
		return mCompressor.getCompressorCurrent() > 0.0 && mPDP.getTotalCurrent() > 0.0;
	}

	public enum ClimbState {
		RETRACTED(SUPERSTRUCTURE.kClimbRetractedState), LOWERED(!SUPERSTRUCTURE.kClimbRetractedState);
		public final boolean state;

		ClimbState(final boolean state) {
			this.state = state;
		}
	}

	public enum RobotState {
		PATH_FOLLOWING, TELEOP_DRIVE, VISION_INTAKE_STATION, VISION_PLACING, VISION_CARGO_INTAKE, VISION_CARGO_OUTTAKE
	}

	private static class InstanceHolder {

		private static final Superstructure mInstance = new Superstructure();
	}
}
