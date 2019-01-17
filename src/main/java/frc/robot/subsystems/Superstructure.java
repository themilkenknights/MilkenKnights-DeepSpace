package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.pixy.Pixy;
import frc.robot.lib.pixy.PixyBlock;
import frc.robot.lib.structure.ILooper;
import frc.robot.lib.structure.Loop;
import frc.robot.lib.vision.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Superstructure extends Subsystem {
	/*public PixySPI pixy1;
	public HashMap<Integer, ArrayList<PixyPacket>> packets = new HashMap<Integer, ArrayList<PixyPacket>>();
	public int w = 0;
	Port port = Port.kOnboardCS0;
	String print;*/
	private LimeLight limeLight;
	private LimelightTarget target;
		static final int FRAME_WIDTH = 320, FRAME_HEIGHT = 200;
		static final int MAX_BLOCKS = 10;

		Pixy pixy1;
		List<Pixy> pixies = new ArrayList<>();

	public static Superstructure getInstance() { return InstanceHolder.mInstance; }

	public Superstructure() { limeLight = new LimeLight();
	limeLight = new LimeLight();
	target = LimelightTarget.EMPTY;
	//pixy1 = new PixySPI(new SPI(port), packets, new PixyException(print));
			pixy1 = new Pixy(0x053C3165);
			pixies.add(pixy1);
			//Pixy.ensureAvailable(0x053C3165, 0xC6E0B552, 0xD892D58D);
			Pixy.ensureAvailable(0xC6E0B552);
			Pixy.enumerate();
	}

	@Override
	public void outputTelemetry() { SmartDashboard.putString("Robot State", Robot.mMatchState.toString()); }

	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) { synchronized (Superstructure.this) {
					for (Pixy pixy : pixies) {
							pixy.setAutoWhiteBalance(false);
							pixy.setWhiteBalanceValue(new Pixy.WhiteBalanceSetting(0xA0, 0x80, 0x80));
							pixy.setAutoExposure(false);
							pixy.setExposureCompensation(new Pixy.ExposureSetting(10, 150));
							pixy.startBlockProgram();
							pixy.startFrameGrabber();
					}

			}
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Superstructure.this) {
					updateLimelight();
						for (Pixy pixy : pixies) {
							for(PixyBlock bl: pixy.getBlocks()){
									System.out.println(bl.x);
							}
						}
				}
			}

			@Override
			public void onStop(double timestamp) {
					for (Pixy pixy : pixies) {
							pixy.stopFrameGrabber();
							pixy.stopBlockProgram();
							pixy.setLEDBrightness(1000);
					}
			}
		});
	}

	private synchronized void updateLimelight() { target = limeLight.returnTarget(); }

	/*private synchronized void testPixy() {
		int ret = -1;
		// Get the packets from the pixy.
		try {
			ret = pixy1.readPackets();
		} catch (PixyException e) {
			e.printStackTrace();
		}
		SmartDashboard.putNumber("Pixy Vision: packets size: ", packets.size());
		for (int i = 1; i <= PixySPI.PIXY_SIG_COUNT; i++) {
			SmartDashboard.putString("Pixy Vision: Signature: ", Integer.toString(i));
			SmartDashboard.putNumber("Pixy Vision: packet: " + i + ": size: ", packets.get(i).size());
			// Loop through the packets for this signature.
			for (int j = 0; j < packets.get(i).size(); j++) {
				System.out.println(packets.get(i).get(j).X);
				SmartDashboard.putNumber("Pixy Vision: " + i + ": X: ", packets.get(i).get(j).X);
				SmartDashboard.putNumber("Pixy Vision: " + i + ": Y: ", packets.get(i).get(j).Y);
				SmartDashboard.putNumber("Pixy Vision: " + i + ": Width: ", packets.get(i).get(j).Width);
				SmartDashboard.putNumber("Pixy Vision: " + i + ": Height: ", packets.get(i).get(j).Height);
			}
		}
	}*/

	public synchronized LimelightTarget getTarget() { return target; }

	private static class InstanceHolder {
		private static final Superstructure mInstance = new Superstructure();
	}
}
