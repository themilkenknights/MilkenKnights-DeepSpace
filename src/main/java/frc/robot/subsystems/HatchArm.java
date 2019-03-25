package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.CAN;
import frc.robot.Constants.HATCH_ARM;
import frc.robot.Constants.MISC;
import frc.robot.Input;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTimer;
import frc.robot.lib.util.Subsystem;
import frc.robot.subsystems.Superstructure.RobotState;

public class HatchArm extends Subsystem {
  private HatchState mHatchState;
  private Solenoid mSpearSolenoid;
  private Solenoid mPancakeSolenoid;
  private boolean mSpearLimitTriggered;
  private NetworkTableEntry mLimitTriggered;
  private NetworkTableEntry mSpearStateTab;
  private NetworkTableEntry mPancakeTab;
  private MkTimer downTimer = new MkTimer();
  private MkTimer autoTimer = new MkTimer();
  private boolean autoHasBeenRun;
  private boolean mPancakeState = false;

  private HatchArm() {
    ShuffleboardTab mHatchArmTab = Shuffleboard.getTab("Hatch Arm");
    mSpearStateTab = mHatchArmTab.add("Spear State", "").getEntry();
    mLimitTriggered = mHatchArmTab.add("Spear Limit", false).getEntry();
    mPancakeTab = mHatchArmTab.add("Pancake", false).getEntry();
    mSpearSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, MISC.kHatchArmChannel);
    mPancakeSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, MISC.kHatchPancakeChannel);
    mHatchState = HatchState.STOW;
  }

  public static HatchArm getInstance() {
    return HatchArm.InstanceHolder.mInstance;
  }

  @Override
  public synchronized void onQuickLoop(double timestamp) {
    mSpearLimitTriggered = CargoArm.getInstance().isSpearLimitTriggered();
    switch (mHatchState) {
      case STOW:
        break;
      case PLACE:
        if (downTimer.isDone() && mSpearLimitTriggered) {
          setPancakeSolenoid(false);
          Input.rumbleDriverController(0.25, 0.5);
          downTimer.reset();
        }
        break;
      case INTAKE:
        if (isHatchLimitTriggered()) {
          setHatchState(HatchState.STOW);
          if (Superstructure.getInstance().getRobotState() == RobotState.TELEOP_DRIVE) {
            Drive.getInstance().setOpenLoop(new DriveSignal(-0.4, -0.4));
          }
        }
        break;
      default:
        Logger.logError("Unexpected Hatch Arm control state: " + mHatchState);
        break;
    }
    mPancakeSolenoid.set(mPancakeState);
    mSpearSolenoid.set(mHatchState.state);
  }

  public void retractPancakeActuator() {
    setPancakeSolenoid(false);
  }

  public boolean isHatchTriggeredTimer() {
    return autoTimer.isDone() && mSpearLimitTriggered;
  }

  public synchronized void setPancakeSolenoid(boolean state) {
    mPancakeState = state;
    Logger.logMarker("Set Pancake State to: " + state);
  }

  public void outputTelemetry(double timestamp) {
    mLimitTriggered.setBoolean(isHatchLimitTriggered());
    mSpearStateTab.setString(mHatchState.toString());
    mPancakeTab.setBoolean(mPancakeSolenoid.get());
  }

  @Override
  public void teleopInit(double timestamp) {

  }

  @Override
  public void autonomousInit(double timestamp) {
    autoHasBeenRun = true;
  }

  @Override
  public void onStop(double timestamp) {
    if (autoHasBeenRun) {
      autoHasBeenRun = false;
    } else {
      setHatchState(HatchState.STOW);
    }
  }

  @Override
  public void onRestart(double timestamp) {

  }

  @Override
  public boolean checkSystem() {
    setHatchState(HatchState.PLACE);
    Logger.logMarker("Set to Place");
    Timer.delay(1.0);
    setHatchState(HatchState.STOW);
    Timer.delay(1.0);
    Logger.logMarker("Set to Stow");
    return true;
  }

  public boolean isHatchLimitTriggered() {
    return mSpearLimitTriggered;
  }

  public synchronized void setHatchState(HatchState state) {
    mHatchState = state;
    switch (state) {
      case PLACE:
        setPancakeSolenoid(true);
        autoTimer.start(0.35);
        downTimer.start(0.4);
        break;
      case STOW:
        setPancakeSolenoid(true);
        autoTimer.reset();
        break;
      case INTAKE:
        autoTimer.start(0.35);
        setPancakeSolenoid(false);
        break;
      default:
        Logger.logErrorWithTrace("Unknown Hatch State");
        break;
    }
    Logger.logMarker("Set Hatch State to " + state.toString());
    mSpearSolenoid.set(state.state);
    autoTimer.start(0.55);
  }

  public HatchState getHatchSpearState() {
    return mHatchState;
  }

  /**
   * The state of the pneumatic spear that places and intakes the Hatches. The default state should always be stowed on power off.
   */
  public enum HatchState {
    PLACE(HATCH_ARM.kHatchArmPlaceState), STOW(!HATCH_ARM.kHatchArmPlaceState), INTAKE(HATCH_ARM.kHatchArmPlaceState);
    public final boolean state;

    HatchState(final boolean state) {
      this.state = state;
    }
  }

  private static class InstanceHolder {
    private static final HatchArm mInstance = new HatchArm();
  }
}
