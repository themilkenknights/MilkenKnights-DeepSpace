package frc.robot.auto.actions;

import frc.robot.AutoChooser;
import frc.robot.Constants.DRIVE;
import frc.robot.RobotState;
import frc.robot.subsystems.Drive;
import frc.robot.util.auto.Action;
import frc.robot.util.auto.trajectory.Path;

public class DrivePathAction implements Action {

    private final Path path;
    private boolean done;
    private boolean brakeMode;

    public DrivePathAction(Path path, boolean dir, boolean flip, boolean brakeMode) {
        this.path = path.copyPath();
        this.brakeMode = brakeMode;
        if (dir) {
            this.path.invert();
        }
        if (flip) {
            this.path.invertSide();
        }
        done = false;
    }

    public DrivePathAction(int pathNum, boolean dir, boolean brakeMode) {
        this(AutoChooser.autoPaths.get(
                "CS-"), dir, false, brakeMode);
    }

    @Override
    public boolean isFinished() {
        if (done) {
            return true;
        }
        if (Drive.getInstance().isPathFinished()) {
            done = true;
            return true;
        }
        return false;
    }

    @Override
    public void update() {

    }

    @Override
    public void done() {
        RobotState.mDriveControlState = RobotState.DriveControlState.VELOCITY_SETPOINT;
    }

    @Override
    public void start() {
        Drive.getInstance().setDrivePath(path, DRIVE.PATH_DIST_TOL, DRIVE.PATH_ANGLE_TOL, brakeMode);
    }
}
