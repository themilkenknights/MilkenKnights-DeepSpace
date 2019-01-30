package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.pixy.PixyException;
import frc.robot.lib.pixy.PixyPacket;
import frc.robot.lib.pixy.PixySPI;
import frc.robot.lib.structure.Subsystem;
import frc.robot.lib.util.Logger;
import frc.robot.lib.util.MovingAverage;
import frc.robot.lib.vision.LimeLight;
import frc.robot.lib.vision.LimeLightControlMode;
import frc.robot.lib.vision.LimelightTarget;
import java.util.ArrayList;
import java.util.HashMap;

public class Vision extends Subsystem {


  private LimeLight limeLight;
  private MovingAverage mThrottleAverage = new MovingAverage(5);
  public PixySPI pixy1;
  Port port = Port.kOnboardCS0;
  String print;
  public HashMap<Integer, ArrayList<PixyPacket>> packets = new HashMap<Integer, ArrayList<PixyPacket>>();
  private Vision() {

    limeLight = new LimeLight();
    pixy1 = new PixySPI(new SPI(port), packets, new PixyException(print));

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

  }

  @Override
  public void onLoop(double timestamp) {
    mThrottleAverage.addNumber(limeLight.returnTarget());
  }

  @Override
  public void onStop(double timestamp) {
    mThrottleAverage.clear();
    limeLight.setLEDMode(LimeLightControlMode.LedMode.kforceOff);
testPixy1();
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

  @Override
  public boolean checkSystem() {
    return limeLight.isConnected();
  }

  public synchronized LimelightTarget getAverageTarget() {
    return mThrottleAverage.getAverage();
  }

  private static class InstanceHolder {

    private static final Vision mInstance = new Vision();
  }
}
