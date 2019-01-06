package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.DriveStraightMode;
import frc.robot.auto.modes.DriveStraightOpenLoopMode;
import frc.robot.subsystems.Drive;
import frc.robot.util.auto.AutoModeBase;
import frc.robot.util.auto.AutoModeExecuter;
import frc.robot.util.auto.trajectory.Path;
import frc.robot.util.logging.Log;

import java.util.HashMap;
import java.util.Map;

public class AutoChooser {

    public static final Map<String, Path> autoPaths = new HashMap<>();
    private static AutoModeExecuter mAutoModeExecuter = null;

    /*
    Initialize Smart Dashboard Choosers and Serialize all paths into memory
     */
    public static void loadAutos() {

    }

    public static AutoModeBase getAutoMode() {
        double delay = SmartDashboard.getNumber("Auto Delay", 0.0);
        if (delay > 0) {
            Timer.delay(delay);
        }
        return getStraightMode();
    }

    private static AutoModeBase getStraightMode() {
        if (Drive.getInstance().isEncodersConnected() && Drive.getInstance().gyroConnected()) {
            return new DriveStraightMode();
        } else {
            return new DriveStraightOpenLoopMode();
        }
    }


    public static void startAuto() {
        updateGameData();
        if (mAutoModeExecuter != null) {
            mAutoModeExecuter.stop();
        }
        mAutoModeExecuter = null;
        mAutoModeExecuter = new AutoModeExecuter();
        mAutoModeExecuter.setAutoMode(getAutoMode());
        mAutoModeExecuter.start();
    }

    public static void disableAuto() {
        if (mAutoModeExecuter != null) {
            mAutoModeExecuter.stop();
        }
        mAutoModeExecuter = null;
    }

    private static void updateGameData() {
        RobotState.matchData.alliance = DriverStation.getInstance().getAlliance();
        RobotState.matchData.matchNumber = DriverStation.getInstance().getMatchNumber();
        RobotState.matchData.matchType = DriverStation.getInstance().getMatchType();
        Log.verbose("Alliance: " + RobotState.matchData.alliance.toString() + " Match Number: "
                + RobotState.matchData.matchNumber + " Match Type: " + RobotState.matchData.matchType
                .toString());
    }

}
