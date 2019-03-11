package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.lib.util.Logger;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;
import java.util.Arrays;

public class Robot extends TimedRobot {

  public static MatchState mMatchState = MatchState.DISABLED;
  private final SubsystemManager mSubsystemManager =
      new SubsystemManager(
          Arrays.asList(
              CargoArm.getInstance(),
              Drive.getInstance(),
              HatchArm.getInstance(),
              Superstructure.getInstance(),
              Vision.getInstance()));

  public Robot() {
    super(Constants.GENERAL.kMainLoopDt);
    Logger.logRobotConstruction();
  }

  @Override
  public void robotInit() {
    try {
      Logger.logRobotInit();
      Shuffleboard.startRecording();
      mSubsystemManager.init();
    } catch (Throwable t) {
      Logger.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledInit() {
    try {
      Logger.logDisabledInit();
      mMatchState = MatchState.DISABLED;
      AutoChooser.disableAuto();
      mSubsystemManager.stop();
    } catch (Throwable t) {
      Logger.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void autonomousInit() {
    try {
      Logger.logAutoInit();
      mMatchState = MatchState.AUTO;
      mSubsystemManager.startAuto();
      AutoChooser.updateGameData();
    } catch (Throwable t) {
      Logger.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void teleopInit() {
    try {
      Logger.logTeleopInit();
      mMatchState = MatchState.TELEOP;
      mSubsystemManager.startTeleop();
    } catch (Throwable t) {
      Logger.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void testInit() {
    try {
      mMatchState = MatchState.TEST;
      mSubsystemManager.startAuto();
      Logger.logMarker("Starting check systems.");
      mSubsystemManager.checkSystem();
      mSubsystemManager.stop();
    } catch (Throwable t) {
      Logger.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void robotPeriodic() {
    try {
      mSubsystemManager.periodicUpdate();
    } catch (Throwable t) {
      Logger.logThrowableCrash(t);
      throw t;
    }
  }

  public enum MatchState {
    AUTO,
    TELEOP,
    DISABLED,
    TEST
  }
}
