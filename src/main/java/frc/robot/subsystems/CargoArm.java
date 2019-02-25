package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.CAN;
import frc.robot.Constants.CARGO_ARM;
import frc.robot.Constants.GENERAL;
import frc.robot.lib.drivers.CT;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;

public class CargoArm extends Subsystem {

    private static CargoArmControlState mCargoArmControlState = CargoArmControlState.MOTION_MAGIC;
    private static CargoArmState mCargoArmState = CargoArmState.ENABLE;
    private final MkTalon mArmTalon;
    private final MkTalon mIntakeTalon;
    private boolean mDisCon, mArmSafety = false;
    private double mStartDis, mOpenLoopSetpoint, mRollerSetpoint, mArmPosEnable = 0.0;
    private NetworkTableEntry mAbsPos, mDesiredState, mControlMode, mStatus, mRawError;

    private CargoArm() {
        ShuffleboardTab mCargoArmTab = Shuffleboard.getTab("Cargo Arm");
        ShuffleboardTab mIntakeRollersTab = Shuffleboard.getTab("Intake Rollers");

        mAbsPos = mCargoArmTab.add("Absolute Pos", 0.0).getEntry();
        mControlMode = mCargoArmTab.add("Control Mode", "").getEntry();
        mStatus = mCargoArmTab.add("Status", false).getEntry();
        mDesiredState = mCargoArmTab.add("Desired State", "").getEntry();
        mRawError = mCargoArmTab.add("Raw Error", 0.0).getEntry();

        mArmTalon = new MkTalon(CAN.kMasterCargoArmTalonID, CAN.kSlaveCargoArmVictorID, TalonLoc.Cargo_Arm, mCargoArmTab);
        mIntakeTalon = new MkTalon(CAN.kLeftCargoIntakeTalonID, CAN.kRightCargoIntakeTalonID, TalonLoc.Cargo_Intake, mIntakeRollersTab);
    }

    public static CargoArm getInstance() {
        return InstanceHolder.mInstance;
    }

    /**
     * The arm encoder was getting periodically briefly disconnected during matches
     * so this method waits 250ms before switching to open loop control.
     *
     * This method also checks for unsafe current output and will automatically
     * switches to open loop control.
     */
    @Override public void safetyCheck(double timestamp) {
        synchronized (CargoArm.this) {
            if (!mArmTalon.isEncoderConnected()) {
                if (mDisCon) {
                    if (Timer.getFPGATimestamp() - mStartDis > 0.25) {
                        setArmControlState(CargoArmControlState.OPEN_LOOP);
                        mDisCon = false;
                        mStartDis = 0;
                    }
                } else {
                    mDisCon = true;
                    mStartDis = Timer.getFPGATimestamp();
                }
                Logger.logError("Cargo Arm Encoder Not Connected");
            } else {
                if (mDisCon) {
                    mDisCon = false;
                    mStartDis = 0;
                    mArmTalon.zeroEncoder();
                    Timer.delay(0.05);
                    setArmControlState(CargoArmControlState.MOTION_MAGIC);
                }
            }


            if (mArmTalon.getCurrent() > CARGO_ARM.kMaxSafeCurrent) {
                Logger.logError("Unsafe Current on Cargo " + mArmTalon.getCurrent() + " Amps");
                setArmControlState(CargoArmControlState.OPEN_LOOP);
            }

        }
    }

    private void setEnable() {
        mArmPosEnable = mArmTalon.getPosition();
        mCargoArmState = CargoArmState.ENABLE;
    }

    /**
     * Configures Soft Limits and Control Mode.
     * Soft limits and open loop mode are the safety/backup state.
     *
     * @param mode False for normal operation. True for safety mode.
     */
    public void enableSafety(boolean mode) {
        if (mode) {
            CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
            CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
            setArmControlState(CargoArmControlState.OPEN_LOOP);
            mArmSafety = true;

        } else {
            CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
            CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
            setEnable();
            setArmControlState(CargoArmControlState.MOTION_MAGIC);
            mArmSafety = false;
        }
    }

    public synchronized void onQuickLoop(double timestamp) {
        if (mCargoArmControlState == CargoArmControlState.OPEN_LOOP) {
            mArmTalon.set(ControlMode.PercentOutput, mOpenLoopSetpoint, NeutralMode.Brake);
        } else if (mCargoArmControlState == CargoArmControlState.MOTION_MAGIC) {
            if (mCargoArmState.equals(CargoArmState.ENABLE)) {
                mArmTalon.set(ControlMode.MotionMagic, MkMath.degreesToNativeUnits(mArmPosEnable), NeutralMode.Brake);
            } else {
                double armFeed = MkMath.sin(mArmTalon.getPosition() + CARGO_ARM.kArmOffset) * CARGO_ARM.kFeedConstant;
                mArmTalon.set(ControlMode.MotionMagic, MkMath.degreesToNativeUnits(mCargoArmState.state), DemandType.ArbitraryFeedForward, -armFeed, NeutralMode.Brake);
            }
        } else {
            Logger.logErrorWithTrace("Unexpected Cargo Arm Control State: " + mCargoArmControlState);
        }

        mIntakeTalon.set(ControlMode.PercentOutput, mRollerSetpoint, NeutralMode.Brake);
    }

    @Override public void outputTelemetry(double timestamp) {
        mArmTalon.updateShuffleboard();
        mAbsPos.setDouble(mArmTalon.masterTalon.getSensorCollection().getPulseWidthPosition());
        mDesiredState.setString(mCargoArmState.toString());
        mControlMode.setString(mCargoArmControlState.toString());
        mStatus.setBoolean(mArmTalon.isEncoderConnected());
        mRawError.setDouble(MkMath.degreesToNativeUnits(mArmTalon.getError()));
    }

    @Override public void teleopInit(double timestamp) {
        mArmTalon.zeroEncoder();
        setEnable();
    }

    @Override public void autonomousInit(double timestamp) {
        mArmTalon.zeroEncoder();
        setEnable();
    }

    @Override public void onStop(double timestamp) {
        setIntakeRollers(0);
    }

    @Override public boolean checkSystem() {
        //TODO Fix return mArmTalon.checkSystem();
        return true;
    }

    public synchronized void setOpenLoop(double output) {
        if (mArmSafety && mCargoArmControlState == CargoArmControlState.OPEN_LOOP) {
            mOpenLoopSetpoint = output;
        } else {
            Logger.logErrorWithTrace("Failed to set Hatch Arm Open Loop Ouput: Arm Safety Not Enabled");
        }
    }

    public synchronized void setIntakeRollers(double output) {
        if (output == CARGO_ARM.kIntakeRollerInSpeed && Vision.getInstance().getPixyTarget().isCargoIntaked()) {
            setArmState(CargoArmState.REVERSE_CARGOSHIP);
        }
        mRollerSetpoint = output;
    }

    public CargoArmControlState getArmControlState() {
        return mCargoArmControlState;
    }

    private synchronized void setArmControlState(CargoArmControlState state) {
        if (state == CargoArmControlState.MOTION_MAGIC && mCargoArmControlState != CargoArmControlState.MOTION_MAGIC) {
            setEnable();
        } else if (state == CargoArmControlState.OPEN_LOOP && mCargoArmControlState != CargoArmControlState.OPEN_LOOP) {
            mOpenLoopSetpoint = 0.0;
        }
        mCargoArmControlState = state;
    }

    public CargoArmState getArmState() {
        return mCargoArmState;
    }

    public synchronized void setArmState(CargoArmState state) {
        if (!mArmSafety) {
            setArmControlState(CargoArmControlState.MOTION_MAGIC);
            mCargoArmState = state;
        } else {
            Logger.logErrorWithTrace("Failed to set Arm State: Arm Safety Enabled");
        }
    }

    public boolean getSafetyState() {
        return mArmSafety;
    }

    public enum CargoArmControlState {
        MOTION_MAGIC, // Closed Loop Motion Profile following on the talons used in nearly all
        // circumstances
        OPEN_LOOP // Direct PercentVBus control of the arm as a failsafe
    }


    public enum CargoArmState {
        ENABLE(0.0), // State directly after robot is enabled (not mapped to a specific angle)
        INTAKE(177.0), FORWARD_ROCKET_LEVEL_ONE(125.0), REVERSE_CARGOSHIP(11.0), REVERSE_ROCKET_LEVEL_TWO(30.0);

        public final double state;

        CargoArmState(final double state) {
            this.state = state;
        }
    }


    private static class InstanceHolder {

        private static final CargoArm mInstance = new CargoArm();
    }
}
