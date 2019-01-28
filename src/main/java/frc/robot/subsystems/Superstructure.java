package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.lib.structure.Subsystem;

public class Superstructure extends Subsystem {
		private PowerDistributionPanel mPDP;
		private Compressor mCompressor;

		private Superstructure() {
				mPDP = new PowerDistributionPanel(Constants.kPowerDistributionPanelID);
				mCompressor = new Compressor();
		}

		@Override
		public void outputTelemetry() {
				SmartDashboard.putString("Robot State", Robot.mMatchState.toString());
				SmartDashboard.putNumber("Compressor Current", mCompressor.getCompressorCurrent());
		}

		@Override
		public void onStart(double timestamp) {

		}

		@Override
		public void onLoop(double timestamp) {

		}

		@Override
		public void onStop(double timestamp) {
		}

		public static Superstructure getInstance() {
				return InstanceHolder.mInstance;
		}


		private static class InstanceHolder {
				private static final Superstructure mInstance = new Superstructure();
		}
}
