package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Constants;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;

public class HatchArm extends Subsystem {
		public HatchArmState mHatchArmState;
		private DoubleSolenoid mArmSolenoid;

		private HatchArm() {
				mArmSolenoid = new DoubleSolenoid(Constants.kHatchArmForwardChannel, Constants.kHatchArmReverseChannel);
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

		public void setHatchArm(HatchArmState armState) {
				mArmSolenoid.set(armState.state);
				Logger.logMarker("Set Hatch Arm to " + armState.toString());
		}


		public enum HatchArmState {
				PLACE(Constants.kHatchArmPlaceState), STOW(Constants.kHatchArmStowState);
				public final DoubleSolenoid.Value state;

				HatchArmState(final DoubleSolenoid.Value state) {
						this.state = state;
				}
		}


		private static class InstanceHolder {
				private static final HatchArm mInstance = new HatchArm();
		}
}
