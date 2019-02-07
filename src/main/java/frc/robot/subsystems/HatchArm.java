package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN;
import frc.robot.Constants.GENERAL;
import frc.robot.Constants.HATCH_ARM;
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
	private HatchArmState mHatchArmState;
	private Solenoid mArmSolenoid;
	private boolean mDisCon = false;
	private double mOpenLoopSetpoint, mArmPosEnable = 0.0;
	private MkTime mStartDis, mTransferTime;

	private HatchArm() {
		mArmSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, HATCH_ARM.kHatchArmChannel);
		mHatchArmState = HatchArmState.STOW;
		mArmTalon = new MkTalon(CAN.kGroundHatchArmTalonID, CAN.kHatchLimitSwitchTalonID, TalonLoc.Hatch_Arm);
		mStartDis = new MkTime();
		mTransferTime = new MkTime();
	}

	public static HatchArm getInstance() {
		return HatchArm.InstanceHolder.mInstance;
	}

	@Override
	public void teleopInit(double timestamp) {
		mArmTalon.zeroEncoder();
		setEnable();
		if (mArmTalon.getZer() > GENERAL.kTicksPerRev) {
			Logger.logMarker("Hatch Arm Absolution Position > 4096");
			setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
		}
	}

	@Override
	public void autonomousInit(double timestamp) {
		mArmTalon.zeroEncoder();
		setEnable();
		if (mArmTalon.getZer() > GENERAL.kTicksPerRev) {
			Logger.logMarker("Hatch Arm Absolution Position > 4096");
			setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
		}
	}

	private void armSafetyCheck() {
		if (!mArmTalon.isEncoderConnected()) {
			if (mDisCon) {
				if (mStartDis.isDone()) {
					setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
					mDisCon = false;
					mStartDis.reset();
				}
			} else {
				mDisCon = true;
				mStartDis.start(0.25);
			}
			Logger.logCriticalError("Hatch Arm Encoder Not Connected");
		} else {
			if (mDisCon) {
				mDisCon = false;
				mStartDis.reset();
				mArmTalon.zeroEncoder();
				Timer.delay(0.05);
				setHatchIntakeControlState(HatchIntakeControlState.MOTION_MAGIC);
			}
		}

		if (mArmTalon.getCurrent() > HATCH_ARM.kMaxSafeCurrent) {
			Logger.logCriticalError("Unsafe Current on Hatch" + mArmTalon.getCurrent() + " Amps");
			setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
		}
	}

	private void setEnable() {
		mArmPosEnable = mArmTalon.getPosition();
		mHatchIntakeState = HatchIntakeState.ENABLE;
	}

	public boolean isHatchLimitTriggered() {
		return mArmTalon.slaveTalon.getSensorCollection().isFwdLimitSwitchClosed();
	}

	/**
	 * @param state False for normal operation. True for safety mode.
	 */
	public void changeSafety(boolean state) {
		if (!state) {
			setHatchMechanismState(HatchMechanismState.UNKNOWN);
			CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
			CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(true, GENERAL.kMediumTimeoutMs));
			setHatchIntakeControlState(HatchIntakeControlState.MOTION_MAGIC);
		} else {
			CT.RE(mArmTalon.masterTalon.configForwardSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
			CT.RE(mArmTalon.masterTalon.configReverseSoftLimitEnable(false, GENERAL.kMediumTimeoutMs));
			setHatchIntakeControlState(HatchIntakeControlState.OPEN_LOOP);
		}
	}

	/*
	Step 3: Write setpoints to Talon
	 */
	@Override
	public synchronized void writePeriodicOutputs(double timestamp) {
		if (mHatchIntakeControlState == HatchIntakeControlState.OPEN_LOOP) {
			mArmTalon.set(ControlMode.PercentOutput, mOpenLoopSetpoint, NeutralMode.Brake);
		} else if (mHatchIntakeControlState == HatchIntakeControlState.MOTION_MAGIC) {
			if (mHatchIntakeState == HatchIntakeState.ENABLE) {
				mArmTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(mArmPosEnable), NeutralMode.Brake);
			} else {
				mArmTalon.set(ControlMode.MotionMagic, MkMath.angleToNativeUnits(mHatchIntakeState.state), NeutralMode.Brake, 0.0);
			}
		} else {
			Logger.logCriticalError("Unexpected arm control state: " + mHatchIntakeControlState);
		}
	}

	@Override
	public void outputTelemetry() {
		//mArmTalon.updateSmartDash(false);
		SmartDashboard.putString("Hatch Arm Desired Position", mHatchIntakeState.toString());
		SmartDashboard.putString("Hatch Arm Control Mode", mHatchIntakeControlState.toString());
		SmartDashboard.putBoolean("Hatch Arm Status", mArmTalon.isEncoderConnected());
		//SmartDashboard.putNumber("Hatch Arm Abs", mArmTalon.getAbsolutePosition());
		SmartDashboard.putString("Hatch Mech State", mHatchMechanismState.toString());
		//SmartDashboard.putNumber("Arm Raw Error", MkMath.angleToNativeUnits(mArmTalon.getError()));
	}


	/**
	 * Updated from mEnabledLoop in Robot.java
	 *
	 * @param timestamp Time in seconds since code start
	 */
	@Override
	public void onLoop(double timestamp) {
		synchronized (HatchArm.this) {
			boolean hatchLimit = isHatchLimitTriggered();
			armSafetyCheck();
			switch (mHatchMechanismState) {
				case STOWED:
					break;
				case GROUND_INTAKE:
					break;
				case PLACING:
					break;
				case STATION_INTAKE:
					if (hatchLimit) {
						setHatchMechanismState(HatchMechanismState.STOWED);
					}
					break;
				case TRANSFER:
					if (hatchLimit) {
						setHatchArmPosition(HatchArmState.STOW);
						mTransferTime.start(0.4);
					}
					if (mTransferTime.isDone()) {
						setHatchMechanismState(HatchMechanismState.STOWED);
						mTransferTime.reset();
					}
					break;
				case MANUAL_OVERRIDE:
					break;
				case UNKNOWN:
					break;
				case VISION_CONTROL:
					break;
				default:
					Logger.logCriticalError("Unexpected Hatch Arm control state: " + mHatchMechanismState);
					break;
			}

		}
	}

	@Override
	public void onStop(double timestamp) {
		setHatchArmPosition(HatchArmState.STOW);
		setHatchMechanismState(HatchMechanismState.UNKNOWN);
	}

	@Override
	public boolean checkSystem() {
		setHatchArmPosition(HatchArmState.PLACE);
		Logger.logMarker("Set to Place");
		Timer.delay(1.0);
		setHatchArmPosition(HatchArmState.STOW);
		Timer.delay(1.0);
		Logger.logMarker("Set to Stow");
		return mArmTalon.checkSystem();
	}

	private void setHatchIntakeControlState(HatchIntakeControlState state) {
		if (state == HatchIntakeControlState.MOTION_MAGIC && mHatchIntakeControlState != HatchIntakeControlState.MOTION_MAGIC) {
			setEnable();
		} else if (state == HatchIntakeControlState.OPEN_LOOP && mHatchIntakeControlState != HatchIntakeControlState.OPEN_LOOP) {
			mOpenLoopSetpoint = 0.0;
		}
		mHatchIntakeControlState = state;
	}

	public synchronized void setOpenLoop(double output) {
		if (mHatchIntakeControlState == HatchIntakeControlState.OPEN_LOOP && mHatchMechanismState == HatchMechanismState.MANUAL_OVERRIDE) {
			mOpenLoopSetpoint = output;
		} else {
			Logger.logCriticalError("Failed to set Hatch Arm Open Loop Ouput: Arm Override Not Enabled");
		}
	}

	private synchronized void setHatchIntakePosition(HatchIntakeState state) {
		if (mHatchMechanismState != HatchMechanismState.MANUAL_OVERRIDE) {
			setHatchIntakeControlState(HatchIntakeControlState.MOTION_MAGIC);
			mHatchIntakeState = state;
		} else {
			Logger.logCriticalError("Failed to set Arm State: Manual Override Enabled");
		}
	}

	/*
	Move Hatch Arm to Stow or Place
	 */
	public synchronized void setHatchArmPosition(HatchArmState armState) {
		mArmSolenoid.set(armState.state);
		//Logger.logMarker("Set Hatch Arm to " + armState.toString());
	}

	public HatchArmState getHatchArmState() {
		return mHatchArmState;
	}

	public HatchMechanismState getHatchMechanismState() {
		return mHatchMechanismState;
	}

	public void setHatchMechanismState(HatchMechanismState state) {
		switch (state) {
			case TRANSFER:
				setHatchArmPosition(HatchArmState.PLACE);
				setHatchIntakePosition(HatchIntakeState.TRANSFER_POINT);
				break;
			case GROUND_INTAKE:
				setHatchIntakePosition(HatchIntakeState.INTAKE_POINT);
				setHatchArmPosition(HatchArmState.STOW);
				break;
			case STOWED:
				setHatchArmPosition(HatchArmState.STOW);
				setHatchIntakePosition(HatchIntakeState.STOW_POINT);
				break;
			case STATION_INTAKE:
				setHatchArmPosition(HatchArmState.PLACE);
				setHatchIntakePosition(HatchIntakeState.STOW_POINT);
				break;
			case PLACING:
				setHatchArmPosition(HatchArmState.PLACE);
				setHatchIntakePosition(HatchIntakeState.STOW_POINT);
				break;
			case MANUAL_OVERRIDE:
				changeSafety(true);
				break;
			case UNKNOWN:
				setEnable();
				break;
			case VISION_CONTROL:
				break;
			default:
				Logger.logCriticalError("Unexpected Hatch Mechanism: " + mHatchMechanismState);
				break;
		}
		mHatchMechanismState = state;
	}


	public enum HatchIntakeControlState {
		MOTION_MAGIC, // Closed Loop Motion Profile following on the talons used in nearly all circumstances
		OPEN_LOOP // Direct PercentVBus control of the arm as a failsafe
	}

	public enum HatchIntakeState {
		ENABLE(0), //State directly after robot is enabled (not mapped to a specific angle)
		INTAKE_POINT(170.0),
		TRANSFER_POINT(75.0), //Outtakes into the switch on the backside of the robot
		STOW_POINT(0.0);

		public final double state;

		HatchIntakeState(final double state) {
			this.state = state;
		}
	}

	public enum HatchArmState {
		PLACE(HATCH_ARM.kHatchArmPlaceState), STOW(!HATCH_ARM.kHatchArmPlaceState);
		public final boolean state;

		HatchArmState(final boolean state) {
			this.state = state;
		}
	}

	public enum HatchMechanismState {
		GROUND_INTAKE, TRANSFER, STATION_INTAKE, STOWED, PLACING, MANUAL_OVERRIDE, UNKNOWN, VISION_CONTROL
	}


	private static class InstanceHolder {

		private static final HatchArm mInstance = new HatchArm();
	}
}
