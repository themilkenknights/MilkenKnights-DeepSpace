package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.CAN;
import frc.robot.Constants.GENERAL;
import frc.robot.Constants.HATCH_ARM;
import frc.robot.Constants.PNUEMATICS;
import frc.robot.lib.drivers.CT;
import frc.robot.lib.drivers.MkTalon;
import frc.robot.lib.drivers.MkTalon.TalonLoc;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTime;

public class HatchArm extends Subsystem {

    private static HatchIntakeControlState mHatchIntakeControlState = HatchIntakeControlState.MOTION_MAGIC;
    private static HatchIntakeState mHatchIntakeState = HatchIntakeState.ENABLE;
    private static HatchMechanismState mHatchMechanismState = HatchMechanismState.UNKNOWN;

    private final MkTalon mArmTalon;
    private HatchSpearState mSpearState;
    private Solenoid mSpearSolenoid;
    private boolean mKetteringDisCon = false;
    private double mKetteringOpenLoopSetpoint, mKetteringPosEnable = 0.0;
    private MkTime mStartDis, mTransferTime, mMoveTime;
    private boolean mSpearLimitTriggered = false;
    private NetworkTableEntry mAbsPos, mDesiredState, mControlMode, mStatus, mRawError, mMechState, mLimitTriggered, mSpearStateTab, mRawPos;

    private HatchArm() {
        ShuffleboardTab mHatchArmTab = Shuffleboard.getTab("Hatch Arm");
        mAbsPos = mHatchArmTab.add("Absolute Pos", 0.0).getEntry();
        mControlMode = mHatchArmTab.add("Control Mode", "").getEntry();
        mStatus = mHatchArmTab.add("Status", false).getEntry();
        mDesiredState = mHatchArmTab.add("Desired State", "").getEntry();
        mRawError = mHatchArmTab.add("Error (Deg)", 0.0).getEntry();
        mSpearStateTab = mHatchArmTab.add("Spear State", "").getEntry();
        mMechState = mHatchArmTab.add("Mechanism State", "").getEntry();
        mLimitTriggered = mHatchArmTab.add("Spear Limit", false).getEntry();
        mRawPos = mHatchArmTab.add("Raw Pos", 0.0).getEntry();

        mSpearSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, PNUEMATICS.kHatchArmChannel);
        mSpearState = HatchSpearState.STOW;
        mArmTalon = new MkTalon(CAN.kGroundHatchArmTalonID, CAN.kKetteringReverseLimitSwitchTalonID, TalonLoc.Hatch_Arm, mHatchArmTab);
        mStartDis = new MkTime();
        mTransferTime = new MkTime();
        mMoveTime = new MkTime();
    }

    public static HatchArm getInstance() {
        return HatchArm.InstanceHolder.mInstance;
    }

    @Override public void teleopInit(double timestamp) {
        mArmTalon.zeroEncoder();
        setEnable();
    }

    @Override public void autonomousInit(double timestamp) {
        mArmTalon.zeroEncoder();
        setEnable();
    }

    @Override public void safetyCheck(double timestamp) {
        if (!mArmTalon.isEncoderConnected()) {
            if (mKetteringDisCon) {
                if (mStartDis.isDone()) {
                    enableSafety(true);
                    mKetteringDisCon = false;
                    mStartDis.reset();
                }
            } else {
                mKetteringDisCon = true;
                mStartDis.start(0.3);
            }
            Logger.logErrorWithTrace("Hatch Arm Encoder Not Connected");
        } else {
            if (mKetteringDisCon) {
                mKetteringDisCon = false;
                mStartDis.reset();
                mArmTalon.zeroEncoder();
                Timer.delay(0.05);
                enableSafety(false);
            }
        }
        if (mArmTalon.getCurrent() > HATCH_ARM.kMaxSafeCurrent) {
            Logger.logErrorWithTrace("Unsafe Current on Hatch" + mArmTalon.getCurrent() + " Amps");
            enableSafety(true);
        }
    }

    private void setEnable() {
        mKetteringPosEnable = mArmTalon.getPosition();
        mHatchIntakeState = HatchIntakeState.ENABLE;
    }

    public boolean isHatchLimitTriggered() {
        return mSpearLimitTriggered;
    }

    /**
     * @param state False for normal operation. True for safety mode.
     */
    public void enableSafety(boolean state) {
        if (state) {
            CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
            CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
            setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
        } else {
            setHatchMechanismState(HatchMechanismState.UNKNOWN);
            CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
            CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
            setHatchIntakeControlState(HatchIntakeControlState.MOTION_MAGIC);
        }
    }


    public void outputTelemetry(double timestamp) {
        mArmTalon.updateShuffleboard();
        mDesiredState.setString(mHatchIntakeState.toString());
        mControlMode.setString(mHatchIntakeControlState.toString());
        mStatus.setBoolean(mArmTalon.isEncoderConnected());
        mAbsPos.setDouble(mArmTalon.masterTalon.getSensorCollection().getPulseWidthPosition());
        mMechState.setString(mHatchMechanismState.toString());
        mRawError.setDouble(MkMath.degreesToNativeUnits(mArmTalon.getError()));
        mRawPos.setDouble(mArmTalon.masterTalon.getSelectedSensorPosition());
        mLimitTriggered.setBoolean(isHatchLimitTriggered());
        mSpearStateTab.setString(mSpearState.toString());
    }

    /**
     * @param timestamp Time in seconds since code start
     */
    @Override public synchronized void onQuickLoop(double timestamp) {
        mSpearLimitTriggered = mArmTalon.slaveTalon.getSensorCollection().isFwdLimitSwitchClosed();
        switch (mHatchMechanismState) {
            case STOWED:
            case SPEAR_STOW_ONLY:
            case SPEAR_PLACE_ONLY:
            case GROUND_INTAKE:
            case PLACING:
            case UNKNOWN:
            case CLEAR_CARGO:
            case VISION_CONTROL:
                break;
            case STATION_INTAKE:
                if (isHatchLimitTriggered()) {
                    setHatchMechanismState(HatchMechanismState.STOWED);
                }
                break;
            case TRANSFER:
                if (mMoveTime.isDone()) {
                    setHatchIntakePosition(HatchIntakeState.TRANSFER_POINT);
                }
                if (isHatchLimitTriggered()) {
                    setHatchSpearState(HatchSpearState.STOW);
                    mTransferTime.start(0.125);
                }
                if (mTransferTime.isDone()) {
                    setHatchMechanismState(HatchMechanismState.STOWED);
                    mTransferTime.reset();
                    mArmTalon.masterTalon.configMotionCruiseVelocity((int) HATCH_ARM.kMotionMagicCruiseVel);
                }
                break;
            default:
                Logger.logErrorWithTrace("Unexpected Hatch Arm control state: " + mHatchMechanismState);
                break;
        }
        if (mHatchIntakeControlState == HatchIntakeControlState.OPEN_LOOP) {
            mArmTalon.set(ControlMode.PercentOutput, mKetteringOpenLoopSetpoint, NeutralMode.Brake);
        } else if (mHatchIntakeControlState == HatchIntakeControlState.MOTION_MAGIC) {
            if (mHatchIntakeState == HatchIntakeState.ENABLE) {
                mArmTalon.set(ControlMode.MotionMagic, MkMath.degreesToNativeUnits(mKetteringPosEnable), NeutralMode.Brake);
            } else {
                mArmTalon.set(ControlMode.MotionMagic, MkMath.degreesToNativeUnits(mHatchIntakeState.state), NeutralMode.Brake);
            }
        } else {
            Logger.logErrorWithTrace("Unexpected arm control state: " + mHatchIntakeControlState);
        }
        mSpearSolenoid.set(mSpearState.state);
    }

    @Override public void onStop(double timestamp) {
        setHatchSpearState(HatchSpearState.STOW);
        setHatchMechanismState(HatchMechanismState.UNKNOWN);
    }

    @Override public boolean checkSystem() {
        //TODO Fix
        /*
        setHatchSpearState(HatchSpearState.PLACE);
        Logger.logMarker("Set to Place");
        Timer.delay(1.0);
        setHatchSpearState(HatchSpearState.STOW);
        Timer.delay(1.0);
        Logger.logMarker("Set to Stow");
        return mArmTalon.checkSystem(); */
        return true;
    }

    public HatchIntakeControlState getHatchIntakeControlState() {
        return mHatchIntakeControlState;
    }

    private void setHatchIntakeControlState(HatchIntakeControlState state) {
        if (state == HatchIntakeControlState.MOTION_MAGIC && mHatchIntakeControlState != HatchIntakeControlState.MOTION_MAGIC) {
            setEnable();
        } else if (state == HatchIntakeControlState.OPEN_LOOP && mHatchIntakeControlState != HatchIntakeControlState.OPEN_LOOP) {
            mKetteringOpenLoopSetpoint = 0.0;
        }
        mHatchIntakeControlState = state;
    }

    public synchronized void setOpenLoop(double output) {
        if (mHatchIntakeControlState == HatchIntakeControlState.OPEN_LOOP) {
            mKetteringOpenLoopSetpoint = output;
        } else {
            Logger.logErrorWithTrace("Failed to set Hatch Arm Open Loop Ouput: Arm Override Not Enabled");
        }
    }

    private synchronized void setHatchIntakePosition(HatchIntakeState state) {
        if (mHatchIntakeControlState == HatchIntakeControlState.MOTION_MAGIC) {
            mHatchIntakeState = state;
        } else {
            Logger.logErrorWithTrace("Failed to set Arm State: Manual Override Enabled");
        }
    }

    public HatchSpearState getHatchSpearState() {
        return mSpearState;
    }

    /*
     * Move Hatch Arm to Stow or Place
     */
    public synchronized void setHatchSpearState(HatchSpearState armState) {
        mSpearState = armState;
        Logger.logMarker("Set Hatch Spear to " + armState.toString());
    }

    public HatchMechanismState getHatchMechanismState() {
        return mHatchMechanismState;
    }

    public void setHatchMechanismState(HatchMechanismState state) {
        if (mHatchMechanismState == HatchMechanismState.TRANSFER && state != HatchMechanismState.TRANSFER) {
            mArmTalon.masterTalon.configMotionCruiseVelocity((int) HATCH_ARM.kMotionMagicCruiseVel);
        } else if (state == HatchMechanismState.TRANSFER) {
            mArmTalon.masterTalon.configMotionCruiseVelocity((int) HATCH_ARM.kMotionMagicCruiseVel / 10);
        }
        switch (state) {
            case TRANSFER:
                setHatchSpearState(HatchSpearState.PLACE);
                mMoveTime.start(0.3);
                break;
            case GROUND_INTAKE:
                setHatchIntakePosition(HatchIntakeState.INTAKE_POINT);
                setHatchSpearState(HatchSpearState.STOW);
                break;
            case STOWED:
                setHatchSpearState(HatchSpearState.STOW);
                setHatchIntakePosition(HatchIntakeState.STOW_POINT);
                break;
            case STATION_INTAKE:
                setHatchSpearState(HatchSpearState.PLACE);
                break;
            case PLACING:
                setHatchSpearState(HatchSpearState.PLACE);
                setHatchIntakePosition(HatchIntakeState.STOW_POINT);
                break;
            case UNKNOWN:
                setEnable();
                break;
            case VISION_CONTROL:
                setHatchSpearState(HatchSpearState.STOW);
                break;
            case SPEAR_STOW_ONLY:
                setHatchSpearState(HatchSpearState.STOW);
                break;
            case SPEAR_PLACE_ONLY:
                setHatchSpearState(HatchSpearState.PLACE);
                break;
            case CLEAR_CARGO:
                setHatchIntakePosition(HatchIntakeState.CLEAR_CARGO_POINT);
                setHatchSpearState(HatchSpearState.STOW);
                break;
            default:
                Logger.logErrorWithTrace("Unexpected Hatch Mechanism: " + mHatchMechanismState);
                break;
        }
        mHatchMechanismState = state;
    }

    public enum HatchIntakeControlState {
        MOTION_MAGIC, // Closed Loop Motion Profile following on the talons used in nearly all circumstances
        OPEN_LOOP // Direct PercentVBus control of the arm as a failsafe
    }


    public enum HatchIntakeState {
        ENABLE(0), // State directly after robot is enabled (not mapped to a specific angle)
        INTAKE_POINT(180), TRANSFER_POINT(55.0), CLEAR_CARGO_POINT(130.0), STOW_POINT(7.0);

        public final double state;

        HatchIntakeState(final double state) {
            this.state = state;
        }
    }


    /**
     * The state of the pneumatic spear that places and intakes the Hatches.
     * The default state should always be stowed on power off.
     */
    public enum HatchSpearState {PLACE(HATCH_ARM.kHatchArmPlaceState), STOW(!HATCH_ARM.kHatchArmPlaceState);
        public final boolean state;

        HatchSpearState(final boolean state) {
            this.state = state;
        }}


    public enum HatchMechanismState {
        GROUND_INTAKE, TRANSFER, STATION_INTAKE, STOWED, PLACING, UNKNOWN, VISION_CONTROL, SPEAR_STOW_ONLY, SPEAR_PLACE_ONLY, CLEAR_CARGO
    }


    private static class InstanceHolder {

        private static final HatchArm mInstance = new HatchArm();
    }
}
