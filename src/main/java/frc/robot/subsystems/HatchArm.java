package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
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

  /*
  Move Hatch Arm to Stow or Place
   */
  public void setHatchArm(HatchArmState armState) {
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
