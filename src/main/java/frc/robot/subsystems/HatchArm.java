package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.CAN;
import frc.robot.Constants.HATCH_ARM;
import frc.robot.Constants.MISC;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.DriveSignal;
import frc.robot.lib.util.Logger;
import frc.robot.subsystems.Superstructure.RobotState;

public class HatchArm extends Subsystem {

  private HatchSpearState mHatchState;
  private Solenoid mSpearSolenoid, mPancakeSolenoid;
  private boolean mSpearLimitTriggered = false;
  private NetworkTableEntry mLimitTriggered, mSpearStateTab;

  private HatchArm() {
    ShuffleboardTab mHatchArmTab = Shuffleboard.getTab("Hatch Arm");
    mSpearStateTab = mHatchArmTab.add("Spear State", "").getEntry();
    mLimitTriggered = mHatchArmTab.add("Spear Limit", false).getEntry();

    mSpearSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, MISC.kHatchArmChannel);
    mPancakeSolenoid = new Solenoid(CAN.kPneumaticsControlModuleID, MISC.kHatchPancakeChannel);
    mHatchState = HatchSpearState.STOW;
  }

  public static HatchArm getInstance() {
    return HatchArm.InstanceHolder.mInstance;
  }

  @Override
  public synchronized void onQuickLoop(double timestamp) {
    mSpearLimitTriggered = CargoArm.getInstance().isSpearLimitTriggered();
    switch (mHatchState) {
      case STOW:
      case PLACE:
        break;
      case INTAKE:
        if (isHatchLimitTriggered()) {
          setHatchState(HatchSpearState.STOW);
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

  public void outputTelemetry(double timestamp) {
    mLimitTriggered.setBoolean(isHatchLimitTriggered());
    mSpearStateTab.setString(mHatchState.toString());
  }

  public boolean isHatchLimitTriggered() {
    return mSpearLimitTriggered;
  }

  @Override
  public void teleopInit(double timestamp) {
  }

  @Override
  public void autonomousInit(double timestamp) {
  }

  @Override
  public void onStop(double timestamp) {
  }

  @Override
  public void onRestart(double timestamp) {

  }

  @Override
  public boolean checkSystem() {
    setHatchState(HatchSpearState.PLACE);
    Logger.logMarker("Set to Place");
    Timer.delay(1.0);
    setHatchState(HatchSpearState.STOW);
    Timer.delay(1.0);
    Logger.logMarker("Set to Stow");
    return true;
  }

  public HatchSpearState getHatchSpearState() {
    return mHatchState;
  }

  public void setHatchState(HatchSpearState state) {
    mSpearSolenoid.set(state.state);
    mHatchState = state;
    switch (state) {
      case PLACE:
      case STOW:
        mPancakeSolenoid.set(true);
      case INTAKE:
        mPancakeSolenoid.set(false);
    }
    Logger.logMarker("Set Hatch State to " + state.toString());
  }


  /**
   * The state of the pneumatic spear that places and intakes the Hatches. The default state should always be stowed on
   * power off.
   */
  public enum HatchSpearState {
    PLACE(HATCH_ARM.kHatchArmPlaceState),
    STOW(!HATCH_ARM.kHatchArmPlaceState),
    INTAKE(HATCH_ARM.kHatchArmPlaceState);
    public final boolean state;

    HatchSpearState(final boolean state) {
      this.state = state;
    }
  }

  private static class InstanceHolder {

    private static final HatchArm mInstance = new HatchArm();
  }
}
