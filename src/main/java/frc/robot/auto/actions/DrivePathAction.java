package frc.robot.auto.actions;

import frc.robot.AutoChooser;
import frc.robot.lib.math.trajectory.Path;
import frc.robot.lib.util.Logger;
import frc.robot.subsystems.Drive;

public class DrivePathAction implements Action {
  private final Path path;
  private int loopCounter = 0;

  public DrivePathAction(Path path, boolean dir) {
    this.path = path.copyPath();
    if (dir) {
      this.path.invert();
    }
  }

  public DrivePathAction(int pathNum, boolean dir) {
    this(AutoChooser.getPath("CS-" + pathNum + (AutoChooser.mAutoPosition == AutoChooser.AutoPosition.LEFT ? "L" : "R")), dir);
  }

  @Override public boolean isFinished() {
    if(loopCounter < 10){
      loopCounter++;
      return false;
    } else{
      return Drive.getInstance().isDriveStateFinished();
    }
  }

  @Override public void update() {

  }

  @Override public void done() {

  }

  @Override public void start() {
    Drive.getInstance().setDrivePath(path, 1.0, 1.0);
  }
}
