package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.modes.*;
import frc.robot.subsystems.Drive;
import frc.robot.util.auto.AutoModeBase;
import frc.robot.util.auto.AutoModeExecuter;
import frc.robot.util.auto.DeserializePath;
import frc.robot.util.auto.trajectory.Path;
import frc.robot.util.logging.Log;

import java.util.HashMap;
import java.util.Map;

public class AutoChooser {

    public static final Map<String, Path> autoPaths = new HashMap<>();
    private static SendableChooser<AutoPosition> positionChooser = new SendableChooser<>();
    private static SendableChooser<AutoAction> actionChooser = new SendableChooser<>();
    private static AutoModeExecuter mAutoModeExecuter = null;

    /*
    Initialize Smart Dashboard Choosers and Serialize all paths into memory
     */
    public static void loadAutos() {
        positionChooser.addDefault("Center", AutoPosition.CENTER);
        positionChooser.addObject("Left", AutoPosition.LEFT);
        positionChooser.addObject("Right", AutoPosition.RIGHT);
        SmartDashboard.putData("Auto Position Chooser", positionChooser);
        actionChooser.addDefault("Switch", AutoAction.SWITCH);
        actionChooser.addObject("Standstill", AutoAction.STANDSTILL);
        actionChooser.addObject("Drive Straight", AutoAction.DRIVE_STRAIGHT);
        SmartDashboard.putData("Auto Action Chooser", actionChooser);
        SmartDashboard.putNumber("Auto Delay", 0.0);
        for (String pathName : Constants.AUTO.autoNames) {
            autoPaths.put(pathName + "LB", DeserializePath.getPathFromFile(pathName + "LB"));
            autoPaths.put(pathName + "LR", DeserializePath.getPathFromFile(pathName + "LR"));
            autoPaths.put(pathName + "RB", DeserializePath.getPathFromFile(pathName + "RB"));
            autoPaths.put(pathName + "RR", DeserializePath.getPathFromFile(pathName + "RR"));
        }
    }

    public static AutoModeBase getAutoMode() {
        double delay = SmartDashboard.getNumber("Auto Delay", 0.0);
        if (delay > 0) {
            Timer.delay(delay);
        }
        Log.verbose(
                "Auto Mode Start: Position - " + RobotState.matchData.robotPosition.toString()
                        + " Action - "
                        + actionChooser.getSelected().toString());
        switch (actionChooser.getSelected()) {
            case STANDSTILL:
                return new StandStillMode();
            case DRIVE_STRAIGHT:
                return getStraightMode();
            case SWITCH:
                return getSwitchMode();
            default:
                Log.marker("Unexpected Auto Mode: " + actionChooser.getSelected().toString() + " + "
                        + positionChooser.getSelected().toString());
                break;
        }
        return null;
    }

    private static AutoModeBase getStraightMode() {
        if (Drive.getInstance().isEncodersConnected() && Drive.getInstance().gyroConnected()) {
            return new DriveStraightMode();
        } else {
            return new DriveStraightOpenLoopMode();
        }
    }

    private static AutoModeBase getSwitchMode() {
        if (positionChooser.getSelected() == AutoPosition.CENTER) {
            return getCenterSwitch();
        } else if (positionChooser.getSelected() == AutoPosition.LEFT) {
            return getLeftSwitch();
        } else if (positionChooser.getSelected() == AutoPosition.RIGHT) {
            return getRightSwitch();
        } else {
            return getStraightMode();
        }
    }

    private static AutoModeBase getCenterSwitch() {
        if (Drive.getInstance().isEncodersConnected() && Drive.getInstance().gyroConnected()) {
            return new CenterSwitchMode();
        } else if (Drive.getInstance().gyroConnected()) {
            return new CenterSwitchOpenLoopGyro();
        } else {
            return new DriveStraightMode();
        }
    }

    private static AutoModeBase getRightSwitch() {
        if (Drive.getInstance().isEncodersConnected() && Drive.getInstance().gyroConnected()) {
            return new RightSwitchMode();
        } else if (RobotState.matchData.switchPosition == GameObjectPosition.RIGHT) {
            return new SwitchOpenLoop();
        } else {
            return getStraightMode();
        }
    }

    private static AutoModeBase getLeftSwitch() {
        if (Drive.getInstance().isEncodersConnected() && Drive.getInstance().gyroConnected()) {
            return new LeftSwitchMode();
        } else if (RobotState.matchData.switchPosition == GameObjectPosition.LEFT) {
            return new SwitchOpenLoop();
        } else {
            return getStraightMode();
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
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        RobotState.matchData.switchPosition =
                gameData.charAt(0) == 'L' ? GameObjectPosition.LEFT : GameObjectPosition.RIGHT;
        RobotState.matchData.scalePosition =
                gameData.charAt(1) == 'L' ? GameObjectPosition.LEFT : GameObjectPosition.RIGHT;
        RobotState.matchData.robotPosition = positionChooser.getSelected();
        Log.verbose("Alliance: " + RobotState.matchData.alliance.toString() + " Match Number: "
                + RobotState.matchData.matchNumber + " Match Type: " + RobotState.matchData.matchType
                .toString() + " " +
                "Switch Position: " + RobotState.matchData.switchPosition.toString() + " Scale Position: "
                + RobotState.matchData.scalePosition.toString());
    }


    public enum AutoPosition {
        LEFT, CENTER, RIGHT
    }

    public enum GameObjectPosition {
        LEFT, RIGHT, INVALID
    }

    public enum AutoAction {
        STANDSTILL, DRIVE_STRAIGHT, SWITCH
    }

}
