package frc.robot.auto.actions;

import frc.robot.subsystems.Drive;

public class WaitForAngle implements Action {

    private double angle;
    public WaitForAngle(double angle){
        this.angle = angle;
    }

    @Override public boolean isFinished() {
      //  return Drive.getInstance().get
        return false;
    }

    @Override public void update() {

    }

    @Override public void done() {

    }

    @Override public void start() {

    }
}
