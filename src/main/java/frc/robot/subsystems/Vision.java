package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.pixy.Pixy;
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
  public Pixy pixy1;

  private Vision() {

    limeLight = new LimeLight();
    pixy1 = new Pixy(Port.kOnboardCS0, 1);

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
    System.out.println(pixy1.getAllDetectedObjects()[0].getCenterX());
  }

  @Override
  public void onStop(double timestamp) {
    mThrottleAverage.clear();
    limeLight.setLEDMode(LimeLightControlMode.LedMode.kforceOff);
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
