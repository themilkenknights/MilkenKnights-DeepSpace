package frc.robot.subsystems;

import frc.robot.lib.pixy.Pixy;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.MovingAverage;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode;
import frc.robot.lib.vision.LimelightTarget;

public class Vision extends Subsystem {
		private LimeLight limeLight;
		private MovingAverage mThrottleAverage = new MovingAverage(5);
		static final int FRAME_WIDTH = 320, FRAME_HEIGHT = 200;
		static final int MAX_BLOCKS = 10;
		Pixy pixy1;
		private Vision() {
			limeLight = new LimeLight();
			pixy1 = new Pixy(0x053C3165);
			Pixy.ensureAvailable(0x053C3165);
		}

		public static Vision getInstance() {
				return Vision.InstanceHolder.mInstance;
		}

		@Override
		public void outputTelemetry() {

		}

		@Override
		public void onStart(double timestamp) {
			limeLight.setLEDMode(LimeLightControlMode.LedMode.kforceOn);
			pixy1.setAutoWhiteBalance(false);
			pixy1.setWhiteBalanceValue(new Pixy.WhiteBalanceSetting(0xA0, 0x80, 0x80));
			pixy1.setAutoExposure(false);
			pixy1.setExposureCompensation(new Pixy.ExposureSetting(10, 150));
			pixy1.startBlockProgram();
			pixy1.startFrameGrabber();
		}

		@Override
		public void onLoop(double timestamp) {
				mThrottleAverage.addNumber(limeLight.returnTarget());
				System.out.println(pixy1.getBlocks().get(0).x);
		}

		@Override
		public void onStop(double timestamp) {
				mThrottleAverage.clear();
				limeLight.setLEDMode(LimeLightControlMode.LedMode.kforceOff);
			pixy1.stopFrameGrabber();
			pixy1.stopBlockProgram();
			pixy1.setLEDBrightness(1000);
		}

		public synchronized LimelightTarget getAverageTarget() {
				return mThrottleAverage.getAverage();
		}

		private static class InstanceHolder {
				private static final Vision mInstance = new Vision();
		}
}
