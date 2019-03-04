package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.AutoChooser;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.MISC;
import frc.robot.Robot;
import frc.robot.auto.modes.ClimbLevel2Mode;
import frc.robot.auto.modes.HatchIntakeVisionPigeon;
import frc.robot.auto.modes.HatchOuttakeVisionPigeon;
import frc.robot.auto.modes.VisionCargoOuttake;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;
import frc.robot.subsystems.CargoArm.CargoArmState;
import frc.robot.subsystems.HatchArm.HatchMechanismState;

public class Superstructure extends Subsystem {

    private static Drive mDrive = Drive.getInstance();
    private static HatchArm mHatch = HatchArm.getInstance();
    private PowerDistributionPanel mPDP;
    private Compressor mCompressor;
    private RobotState mRobotState = RobotState.TELEOP_DRIVE;
    private Solenoid mFrontClimbSolenoid, mRearClimbSolenoid;
    private ClimbState mRearClimbState = ClimbState.RETRACTED;
    private ClimbState mFrontClimbState = ClimbState.RETRACTED;
    private NetworkTableEntry mMatchState, mRobotStateEntry, mCompressorCurrent, mFrontClimb, mRearClimb;

    /**
     * Stores PDP, Compressor, General Robot Data, and Climb Actuators.
     * Acts as the high-level controller that changes calls the lower-level subsystems.
     */
    private Superstructure() {
        ShuffleboardTab mStructureTab = Shuffleboard.getTab("Superstructure");
        mMatchState = mStructureTab.add("Match State", "").getEntry();
        mRobotStateEntry = mStructureTab.add("Robot State", "").getEntry();
        mCompressorCurrent = mStructureTab.add("Compressor Current", 0.0).getEntry();
        mFrontClimb = mStructureTab.add("Front Climb", "").getEntry();
        mRearClimb = mStructureTab.add("Rear Climb", "").getEntry();

        mFrontClimbSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, MISC.kFrontClimbSolenoidChannel);
        mRearClimbSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, MISC.kRearClimbSolenoidChannel);

        mPDP = new PowerDistributionPanel(Constants.CAN.kPowerDistributionPanelID);
        LiveWindow.disableTelemetry(mPDP);
        mCompressor = new Compressor(CAN.kPneumaticsControlModuleID);
    }

    public static Superstructure getInstance() {
        return InstanceHolder.mInstance;
    }

    @Override public void outputTelemetry(double timestamp) {
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

    public void teleopInit(double timestamp) {
        setRobotState(RobotState.TELEOP_DRIVE);
    }

    @Override public void autonomousInit(double timestamp) {
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
                Vision.getInstance().disableLED();
                break;
            case HATCH_VISION_INTAKE:
                startVisionHatchIntake();
            case HATCH_VISION_OUTTAKE:
                startVisionHatchOuttake();
                break;
            case VISION_CARGO_OUTTAKE:
                startVisionCargoOuttake();
            case PATH_FOLLOWING:
                break;
            case AUTO_CLIMB:
                startAutoClimb();
                break;
            default:
                Logger.logErrorWithTrace("Unexpected robot state: " + mRobotState);
                break;
        }
        Logger.logMarker("Switching to Robot State:" + mRobotState);
    }

    private void startVisionHatchOuttake() {
        Vision.getInstance().enableLED();
        Timer.delay(0.02);
        Vision.getInstance().updateLimelight();
        if (!Vision.getInstance().getLimelightTarget().isValidTarget()) {
            setRobotState(RobotState.TELEOP_DRIVE);
            Logger.logMarker("Limelight target not valid");
        } else {
            AutoChooser.startAuto(new HatchOuttakeVisionPigeon());
        }
    }

    private void startVisionHatchIntake() {
        Vision.getInstance().enableLED();
        if (!Vision.getInstance().getLimelightTarget().isValidTarget()) {
            setRobotState(RobotState.TELEOP_DRIVE);
            Logger.logMarker("Limelight target not valid");
        }
        AutoChooser.startAuto(new HatchIntakeVisionPigeon());
    }

    private void startVisionCargoOuttake() {
        Vision.getInstance().enableLED();
        Timer.delay(1.0);
        //mRobotState = Vision.getInstance().getLimelightTarget().isValidTarget() ? mRobotState : RobotState.TELEOP_DRIVE;
        mHatch.setHatchMechanismState(HatchMechanismState.STOWED);
        CargoArm.getInstance().setArmState(CargoArmState.REVERSE_CARGOSHIP);
        AutoChooser.startAuto(new VisionCargoOuttake());
    }

    private void startAutoClimb() {
        AutoChooser.startAuto(new ClimbLevel2Mode());
    }

    @Override public void onStop(double timestamp) {
        setFrontClimbState(ClimbState.RETRACTED);
        setRearClimbState(ClimbState.RETRACTED);
    }

    @Override public boolean checkSystem() {
        return mCompressor.getCompressorCurrent() > 0.0 && mPDP.getTotalCurrent() > 0.0 && mPDP.getVoltage() > 0.0;
    }

    public enum ClimbState {
        RETRACTED(false), LOWERED(true);
        public final boolean state;

        ClimbState(final boolean state) {
            this.state = state;
        }
    }


    public enum RobotState {
        PATH_FOLLOWING, TELEOP_DRIVE, HATCH_VISION_INTAKE, HATCH_VISION_OUTTAKE, VISION_CARGO_OUTTAKE, AUTO_CLIMB
    }


    private static class InstanceHolder {

        private static final Superstructure mInstance = new Superstructure();
    }
}
