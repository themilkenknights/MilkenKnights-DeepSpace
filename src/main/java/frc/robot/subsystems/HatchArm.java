package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;

public class HatchArm extends Subsystem {

  public HatchArmState mHatchArmState;
  private Solenoid mArmSolenoid;

  private HatchArm() {
    mArmSolenoid = new Solenoid(Constants.HATCH.kHatchArmForwardChannel,
        Constants.HATCH.kHatchArmReverseChannel);
    mHatchArmState = HatchArmState.STOW;
  }

  public static HatchArm getInstance() {
    return HatchArm.InstanceHolder.mInstance;
  }

  @Override
  public void outputTelemetry() {
  }

  @Override
  public void onStart(double timestamp) {
  }

  @Override
  public void onStop(double timestamp) {
    setHatchArm(HatchArmState.STOW);
  }

  @Override
  public boolean checkSystem() {
    setHatchArm(HatchArmState.PLACE);
    Logger.logMarker("Set to Place");
    Timer.delay(1.0);
    setHatchArm(HatchArmState.STOW);
    Timer.delay(1.0);
    Logger.logMarker("Set to Stow");
    return true;
  }

  /*
  Move Hatch Arm to Stow or Place
   */
  public synchronized void setHatchArm(HatchArmState armState) {
    mArmSolenoid.set(armState.state);
    Logger.logMarker("Set Hatch Arm to " + armState.toString());
  }

  public enum HatchArmState {
    PLACE(Constants.HATCH.kHatchArmPlaceState), STOW(!Constants.HATCH.kHatchArmPlaceState);
    public final boolean state;

    HatchArmState(final boolean state) {
      this.state = state;
    }
  }


  private static class InstanceHolder {

    private static final HatchArm mInstance = new HatchArm();
  }
}
