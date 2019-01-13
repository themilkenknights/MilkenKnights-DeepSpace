package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.structure.ILooper;
import frc.robot.lib.structure.Loop;
import frc.robot.lib.vision.*;

import java.util.ArrayList;
import java.util.HashMap;

public class Superstructure extends Subsystem {
		public PixySPI pixy1;
		public HashMap<Integer, ArrayList<PixyPacket>> packets = new HashMap<Integer, ArrayList<PixyPacket>>();
		public int w = 0;
		Port port = Port.kOnboardCS0;
		String print;
		private LimeLight limeLight;
		private LimelightTarget target;

		public static Superstructure getInstance() {
				return InstanceHolder.mInstance;
		}

		public void Superstructure() {
				limeLight = new LimeLight();
				target = LimelightTarget.EMPTY;
				pixy1 = new PixySPI(new SPI(port), packets, new PixyException(print));
		}

		@Override public void outputTelemetry() {
				SmartDashboard.putString("Robot State", Robot.mMatchState.toString());
		}

		public void registerEnabledLoops(ILooper enabledLooper) {
				enabledLooper.register(new Loop() {
						@Override public void onStart(double timestamp) {
								synchronized (Superstructure.this) {
								}
						}

						@Override public void onLoop(double timestamp) {
								synchronized (Superstructure.this) {
										updateLimelight();
										testPixy();
								}
						}

						@Override public void onStop(double timestamp) {
						}
				});
		}

		private synchronized void updateLimelight() {
				target = limeLight.returnTarget();
		}

		private synchronized void testPixy() {
				if (w == 500) {
						w = 0;
				} else {
						w++;
						return;
				}
				int ret = -1;
				// Get the packets from the pixy.
				try {
						ret = pixy1.readPackets();
				} catch (PixyException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
				}
				SmartDashboard.putNumber("Pixy Vision: packets size: ", packets.size());
				for (int i = 1; i <= PixySPI.PIXY_SIG_COUNT; i++) {
						SmartDashboard.putString("Pixy Vision: Signature: ", Integer.toString(i));
						SmartDashboard.putNumber("Pixy Vision: packet: " + i + ": size: ", packets.get(i).size());
						// Loop through the packets for this signature.
						for (int j = 0; j < packets.get(i).size(); j++) {
								SmartDashboard.putNumber("Pixy Vision: " + i + ": X: ", packets.get(i).get(j).X);
								SmartDashboard.putNumber("Pixy Vision: " + i + ": Y: ", packets.get(i).get(j).Y);
								SmartDashboard.putNumber("Pixy Vision: " + i + ": Width: ", packets.get(i).get(j).Width);
								SmartDashboard.putNumber("Pixy Vision: " + i + ": Height: ", packets.get(i).get(j).Height);
						}
				}
		}

		public synchronized LimelightTarget getTarget() {
				return target;
		}

		private static class InstanceHolder {
				private static final Superstructure mInstance = new Superstructure();
		}
}
