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
import frc.robot.Constants.MISC;
import frc.robot.lib.drivers.CT;
import frc.robot.lib.math.MkMath;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTime;
import frc.robot.misc.MkTalon;
import frc.robot.misc.MkTalon.TalonLoc;
import frc.robot.subsystems.Superstructure.RobotState;

public class HatchArm extends Subsystem {

    private static mKetteringControlState mHatchIntakeControlState = mKetteringControlState.MOTION_MAGIC;
    private static mKetteringSetpoint mHatchKetteringSetpoint = mKetteringSetpoint.ENABLE;
    private static HatchMechanismState mHatchMechanismState = HatchMechanismState.UNKNOWN;

    private final MkTalon mArmTalon;
    private HatchSpearState mSpearState;
    private Solenoid mSpearSolenoid;
    private boolean mKetteringDisCon = false;
    private double mKetteringOpenLoopSetpoint, mKetteringPosEnable = 0.0;
    private MkTime mStartDis, mMoveTime, fastTime;
    private boolean mSoftLimitState = true;
    private boolean mSpearLimitTriggered = false;
    private NetworkTableEntry mAbsPos, mDesiredState, mControlMode, mStatus, mRawError, mMechState, mLimitTriggered, mSpearStateTab, mRawPos;
    private boolean first = true;
    private boolean second = true;

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

        mSpearSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, MISC.kHatchArmChannel);
        mSpearState = HatchSpearState.STOW;
        mArmTalon = new MkTalon(CAN.kGroundHatchArmTalonID, CAN.kKetteringReverseLimitSwitchTalonID, TalonLoc.Hatch_Arm, mHatchArmTab);
        mStartDis = new MkTime();
        mMoveTime = new MkTime();
        fastTime = new MkTime();
    }

    public static HatchArm getInstance() {
        return HatchArm.InstanceHolder.mInstance;
    }

    @Override public void teleopInit(double timestamp) {
        zeroEncoder();
        setHatchSpearState(HatchSpearState.STOW);
    }

    @Override public void autonomousInit(double timestamp) {
        zeroEncoder();
        setHatchSpearState(HatchSpearState.STOW);
    }

    @Override public void safetyCheck(double timestamp) {
        mArmTalon.checkForError();
        if (!mArmTalon.isEncoderConnected()) {
            if (mKetteringDisCon) {
                if (mStartDis.isDone()) {
                    setOpenLoop(0.0);
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
            }
        }

        if (mArmTalon.getCurrent() > HATCH_ARM.kMaxSafeCurrent) {
            Logger.logErrorWithTrace("Unsafe Current on Hatch" + mArmTalon.getCurrent() + " Amps");
            setOpenLoop(0.0);
        }
    }

    private void setEnable() {
        mKetteringPosEnable = mArmTalon.getPosition();
        mHatchKetteringSetpoint = mKetteringSetpoint.ENABLE;
    }

    public boolean isHatchLimitTriggered() {
        return mSpearLimitTriggered;
    }

    public void disableSoftLimit() {
        CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(false, GENERAL.kShortTimeoutMs));
        CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(false, GENERAL.kShortTimeoutMs));
    }

    public boolean isKetteringReverseTriggered() {
        return mArmTalon.slaveTalon.getSensorCollection().isRevLimitSwitchClosed();
    }

    public void outputTelemetry(double timestamp) {
        mArmTalon.updateShuffleboard();
        mDesiredState.setString(mHatchKetteringSetpoint.toString());
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
        mSpearLimitTriggered = CargoArm.getInstance().spearLimit();
        switch (mHatchMechanismState) {
            case STOWED:
            case SPEAR_STOW_ONLY:
            case SPEAR_PLACE_ONLY:
            case GROUND_INTAKE:
            case PLACING:
            case UNKNOWN:
            case VISION_CONTROL:
                break;
            case STATION_INTAKE:
                if (isHatchLimitTriggered()) {
                    setHatchMechanismState(HatchMechanismState.STOWED);
                    if (Superstructure.getInstance().getRobotState() == RobotState.TELEOP_DRIVE) {
                        Drive.getInstance().setOpenLoop(new DriveSignal(-0.4, -0.4));
                    }
                }
                break;
            case TRANSFER:
                if (mMoveTime.isDone() && second) {
                    setHatchIntakePosition(mKetteringSetpoint.TRANSFER_POINT);
                    second = false;
                } else if (fastTime.isDone()) {
                    setHatchMechanismState(HatchMechanismState.STOWED);
                    fastTime.reset();
                    second = true;
                    first = true;
                } else if (isHatchLimitTriggered() && first) {
                    mArmTalon.masterTalon.configMotionCruiseVelocity((int) HATCH_ARM.kMotionMagicCruiseVel);
                    setHatchSpearState(HatchSpearState.STOW);
                    setHatchIntakePosition(mKetteringSetpoint.INTAKE_POINT);
                    first = false;
                    fastTime.start(0.2);
                }
                break;
            default:
                Logger.logErrorWithTrace("Unexpected Hatch Arm control state: " + mHatchMechanismState);
                break;
        }
        if (mHatchIntakeControlState == mKetteringControlState.OPEN_LOOP) {
            mArmTalon.set(ControlMode.PercentOutput, mKetteringOpenLoopSetpoint, NeutralMode.Brake);
        } else if (mHatchIntakeControlState == mKetteringControlState.MOTION_MAGIC) {
            if (mHatchKetteringSetpoint == mKetteringSetpoint.ENABLE) {
                mArmTalon.set(ControlMode.MotionMagic, MkMath.degreesToNativeUnits(mKetteringPosEnable), NeutralMode.Brake);
            } else {
                mArmTalon.set(ControlMode.MotionMagic, MkMath.degreesToNativeUnits(mHatchKetteringSetpoint.state), NeutralMode.Brake);
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

    @Override public void onRestart(double timestamp) {
        mArmTalon.checkForError();
    }

    @Override public boolean checkSystem() {
        setHatchSpearState(HatchSpearState.PLACE);
        Logger.logMarker("Set to Place");
        Timer.delay(1.0);
        setHatchSpearState(HatchSpearState.STOW);
        Timer.delay(1.0);
        Logger.logMarker("Set to Stow");
        return mArmTalon.checkSystem();
    }

    private void setHatchIntakeControlState(mKetteringControlState state) {
        if (state == mKetteringControlState.MOTION_MAGIC && mHatchIntakeControlState != mKetteringControlState.MOTION_MAGIC) {
            setEnable();
        } else if (state == mKetteringControlState.OPEN_LOOP && mHatchIntakeControlState != mKetteringControlState.OPEN_LOOP) {
            mKetteringOpenLoopSetpoint = 0.0;
        }
        mHatchIntakeControlState = state;
    }

    public synchronized void setOpenLoop(double output) {
        if (mHatchIntakeControlState != mKetteringControlState.OPEN_LOOP) {
            Logger.logMarker("Switching to Hatch Arm Open Loop");
            setHatchIntakeControlState(mKetteringControlState.OPEN_LOOP);
        }
        mKetteringOpenLoopSetpoint = output;
    }

    private synchronized void setHatchIntakePosition(mKetteringSetpoint state) {
        if (mHatchIntakeControlState != mKetteringControlState.MOTION_MAGIC) {
            Logger.logMarker("Switching to Hatch Arm Motion Magic");
            setHatchIntakeControlState(mKetteringControlState.MOTION_MAGIC);
        }
        mHatchKetteringSetpoint = state;
    }

    public HatchSpearState getHatchSpearState() {
        return mSpearState;
    }

    /**
     * Move Spear up to stow or down to place
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
            mArmTalon.masterTalon.configMotionCruiseVelocity((int) HATCH_ARM.kMotionMagicCruiseVel / 9);
        }
        mHatchMechanismState = state;
        switch (state) {
            case TRANSFER:
                setHatchSpearState(HatchSpearState.PLACE);
                mMoveTime.start(0.3);
                break;
            case GROUND_INTAKE:
                setHatchIntakePosition(mKetteringSetpoint.INTAKE_POINT);
                setHatchSpearState(HatchSpearState.STOW);
                break;
            case STOWED:
                setHatchSpearState(HatchSpearState.STOW);
                setHatchIntakePosition(mKetteringSetpoint.STOW_POINT);
                break;
            case STATION_INTAKE:
                setHatchSpearState(HatchSpearState.PLACE);
                break;
            case PLACING:
                setHatchSpearState(HatchSpearState.PLACE);
                setHatchIntakePosition(mKetteringSetpoint.STOW_POINT);
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
            default:
                Logger.logErrorWithTrace("Unexpected Hatch Mechanism: " + mHatchMechanismState);
                break;
        }
    }

    public void zeroEncoder() {
        mArmTalon.zeroEncoder();
        setEnable();
    }


    public enum mKetteringControlState {
        MOTION_MAGIC, // Closed Loop Motion Profile following on the talons used in nearly all circumstances
        OPEN_LOOP // Direct PercentVBus control of the arm as a failsafe
    }


    public enum mKetteringSetpoint {
        ENABLE(0), // State directly after robot is enabled (not mapped to a specific angle)
        INTAKE_POINT(178.0), TRANSFER_POINT(57.5), STOW_POINT(4.0);

        public final double state;

        mKetteringSetpoint(final double state) {
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
        GROUND_INTAKE, TRANSFER, STATION_INTAKE, STOWED, PLACING, UNKNOWN, VISION_CONTROL, SPEAR_STOW_ONLY, SPEAR_PLACE_ONLY
    }


    private static class InstanceHolder {

        private static final HatchArm mInstance = new HatchArm();
    }
}
