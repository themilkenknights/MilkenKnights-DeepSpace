package frc.robot.auto.actions;

import frc.robot.AutoChooser;
import frc.robot.lib.util.DeserializePath;
import frc.robot.lib.math.trajectory.Path;
import frc.robot.subsystems.Drive;

public class DrivePathAction implements Action {
  private final Path path;

  public DrivePathAction(Path path, boolean dir) {
    this.path = path;
    if (dir) {
      this.path.invert();
    }
  }

  public DrivePathAction(int pathNum, boolean dir) {
    this(AutoChooser.autoPaths.get("CS-" + pathNum + (AutoChooser.mAutoPosition == AutoChooser.AutoPosition.LEFT ? "L" : "R")), dir);
  }

  @Override public boolean isFinished() {
    return Drive.getInstance().isDriveStateFinished();
  }

  @Override public void update() {

  }

  @Override public void done() {

  }

  @Override public void start() {
    Drive.getInstance().setDrivePath(path, 1.0, 1.0);
  }
}
