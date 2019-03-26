package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.CAN;
import frc.robot.Constants.MISC;
import frc.robot.Input;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MkTimer;
import frc.robot.lib.util.Subsystem;
import frc.robot.subsystems.Superstructure.RobotState;

public class HatchArm extends Subsystem {
  private HatchState mHatchState;
  private Solenoid mFirstSpearSolenoid;
  private Solenoid mSecondSpearSolenoid;
  private Solenoid mPancakeSolenoid;
  private boolean mSpearLimitTriggered;
  private NetworkTableEntry mLimitTriggered;
  private NetworkTableEntry mSpearStateTab;
  private NetworkTableEntry mPancakeTab;
  private MkTimer downTimer = new MkTimer();
  private boolean pancakeState;
  private boolean hasBeenRumbled;

  private HatchArm() {
    ShuffleboardTab mHatchArmTab = Shuffleboard.getTab("Hatch Arm");
    mSpearStateTab = mHatchArmTab.add("Spear State", "").getEntry();
    mLimitTriggered = mHatchArmTab.add("Spear Limit", false).getEntry();
    mPancakeTab = mHatchArmTab.add("Pancake", false).getEntry();
    mFirstSpearSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, MISC.kFirstHatchArmChannel);
    mSecondSpearSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, MISC.kSecondHatchArmChannel);
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
        if (downTimer.isDone(0.4) && mSpearLimitTriggered && !hasBeenRumbled) {
          Logger.logMarker("PLACE LIMIT TRIGGERED");
          setPancakeSolenoid(false);
          Input.rumbleDriverController(0.25, 0.5);
          hasBeenRumbled = true;
        }
        break;
      case INTAKE:
        if (mSpearLimitTriggered && downTimer.isDone(0.25) && !hasBeenRumbled) {
          hasBeenRumbled = true;
          Logger.logMarker("INTAKE LIMIT TRIGGERED");
          if (Superstructure.getInstance().getRobotState() != RobotState.CARGOSHIP_AUTO) {
            setHatchState(HatchState.STOW);
          }
          Input.rumbleDriverController(0.25, 0.25);
          if (Superstructure.getInstance().getRobotState() == RobotState.TELEOP_DRIVE) {
            Drive.getInstance().setOpenLoop(new DriveSignal(-0.4, -0.4));
          }
        }
        break;
      default:
        Logger.logError("Unexpected Hatch Arm control state: " + mHatchState);
        break;
    }
  }

  public synchronized void retractPancakeActuator() {
    setPancakeSolenoid(false);
  }

  public synchronized boolean isHatchTriggeredTimer() {
    /*Logger.logMarker("Rumble: " + hasBeenRumbled);
    if(hasBeenRumbled || (downTimer.isDone(0.2) && mSpearLimitTriggered)){
        Logger.logMarker("!!!! Rumble: " + hasBeenRumbled + " downTime: " + downTimer.isDone(0.4) + " Spear: " + mSpearLimitTriggered);
    }*/
    return downTimer.isDone(0.4) && mSpearLimitTriggered;
  }

  public synchronized void setPancakeSolenoid(boolean state) {
    mPancakeSolenoid.set(state);
    pancakeState = state;
    Logger.logMarker("Set Pancake State to: " + state);
  }

  public void outputTelemetry(double timestamp) {
    mLimitTriggered.setBoolean(isHatchLimitTriggered());
    mSpearStateTab.setString(mHatchState.toString());
    mPancakeTab.setBoolean(pancakeState);
  }

  @Override
  public void teleopInit(double timestamp) {

  }

  @Override
  public void autonomousInit(double timestamp) {

  }

  @Override
  public void onStop(double timestamp) {
    hasBeenRumbled = false;
    downTimer.reset();
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
    if ((mHatchState == HatchState.STOW) && (state != HatchState.STOW)) {
      downTimer.start(0.4);
    }
    mHatchState = state;
    switch (state) {
      case PLACE:
        setPancakeSolenoid(true);
        break;
      case STOW:
        downTimer.reset();
        setPancakeSolenoid(true);
        break;
      case INTAKE:
        setPancakeSolenoid(false);
        break;
      default:
        Logger.logErrorWithTrace("Unknown Hatch State");
        break;
    }
    hasBeenRumbled = false;
    Logger.logMarker("Set Hatch State to " + state.toString());
    mFirstSpearSolenoid.set(state.firstState);
    mSecondSpearSolenoid.set(state.secondState);
  }

  public HatchState getHatchSpearState() {
    return mHatchState;
  }

  /**
   * The state of the pneumatic spear that places and intakes the Hatches. The default state should always be stowed on power off.
   */
  public enum HatchState {
    PLACE(true, false),
    STOW(false, true),
    INTAKE(true, false);
    public final boolean firstState;
    public final boolean secondState;

    HatchState(final boolean firstState, final boolean secondState) {
      this.firstState = firstState;
      this.secondState = secondState;
    }
  }

  private static class InstanceHolder {
    private static final HatchArm mInstance = new HatchArm();
  }
}
