package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.structure.ILooper;
import frc.robot.lib.structure.Loop;
import frc.robot.lib.vision.PixyException;
import frc.robot.lib.vision.PixyPacket;
import frc.robot.lib.vision.PixySPI;
import java.util.ArrayList;
import java.util.HashMap;

public class Superstructure extends Subsystem {

	public PixySPI pixy1;
	Port port = Port.kOnboardCS0;
	String print;
	public HashMap<Integer, ArrayList<PixyPacket>> packets = new HashMap<Integer, ArrayList<PixyPacket>>();

	public void Superstructure(){
		pixy1 = new PixySPI(new SPI(port), packets, new PixyException(print));

	}

	public static Superstructure getInstance() {
		return InstanceHolder.mInstance;
	}

	@Override
	public void outputTelemetry() {
		SmartDashboard.putString("Robot State", Robot.mMatchState.toString());
	}


	public void registerEnabledLoops(ILooper enabledLooper) {
		enabledLooper.register(new Loop() {

			@Override
			public void onStart(double timestamp) {
				synchronized (Superstructure.this) {
				}
			}

			@Override
			public void onLoop(double timestamp) {
				synchronized (Superstructure.this) {
				}
			}

			@Override
			public void onStop(double timestamp) {

			}
		});
	}

	private static class InstanceHolder {

		private static final Superstructure mInstance = new Superstructure();

	}

	public void testPixy1(){
		int ret = -1;
		// Get the packets from the pixy.
		try {
			ret = pixy1.readPackets();
		} catch (PixyException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		SmartDashboard.putNumber("Pixy Vision: packets size: ", packets.size());

		for(int i = 1; i <= PixySPI.PIXY_SIG_COUNT ; i++) {
			SmartDashboard.putString("Pixy Vision: Signature: ", Integer.toString(i));

			SmartDashboard.putNumber("Pixy Vision: packet: " + Integer.toString(i) + ": size: ", packets.get(i).size());

			// Loop through the packets for this signature.
			for(int j=0; j < packets.get(i).size(); j++) {
				SmartDashboard.putNumber("Pixy Vision: " + Integer.toString(i) + ": X: ", packets.get(i).get(j).X);
				SmartDashboard.putNumber("Pixy Vision: " + Integer.toString(i) + ": Y: ", packets.get(i).get(j).Y);
				SmartDashboard.putNumber("Pixy Vision: " + Integer.toString(i) + ": Width: ", packets.get(i).get(j).Width);
				SmartDashboard.putNumber("Pixy Vision: " + Integer.toString(i) + ": Height: ", packets.get(i).get(j).Height);
			}
		}
	}
}
