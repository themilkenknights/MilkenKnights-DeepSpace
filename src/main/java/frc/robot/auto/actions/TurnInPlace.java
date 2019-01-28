package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class TurnInPlace implements Action{

  private double angle;
  private double desiredTime,startTime = 0;

  public TurnInPlace(double angle, double desiredTime){
    this.angle = angle;
    this.desiredTime = desiredTime;
  }
  @Override
  public boolean isFinished() {
    return Drive.getInstance().isTurnDone() || (Timer.getFPGATimestamp() - startTime) > desiredTime;
  }

  @Override
  public void update() {

  }

  @Override
  public void done() {
    Drive.getInstance().setOpenLoop(DriveSignal.BRAKE);
  }

  @Override
  public void start() {
    System.out.println("Start Turn In Place");
    Drive.getInstance().startTurnInPlace(angle);
  }
}
