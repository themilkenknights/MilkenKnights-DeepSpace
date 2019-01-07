/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.util.structure.SubsystemManager;
import frc.robot.RobotState.DriveControlState;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.logging.Log;
import frc.robot.util.structure.loops.Looper;
import java.util.Arrays;

public class Robot extends TimedRobot {
  private final SubsystemManager mSubsystemManager = new SubsystemManager(
      Arrays.asList(Drive.getInstance(), Superstructure.getInstance(), Input.getInstance()));
  private Looper mEnabledLooper = new Looper();

  public Robot() {
    Log.logRobotStartup();
  }

  @Override
  public void robotInit() {
    try {
      Log.logRobotInit();
      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
    } catch (Throwable t) {
      Log.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledInit() {
    try {
      Log.logDisabledInit();
      mEnabledLooper.stop();
      AutoChooser.disableAuto();
      RobotState.resetDefaultState();
    } catch (Throwable t) {
      Log.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void autonomousInit() {
    try {
      Log.logAutoInit();
      RobotState.mDriveControlState = DriveControlState.VISION_TRACKING;
      mEnabledLooper.start();
    } catch (Throwable t) {
      Log.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    try {
      Log.logTeleopInit();
      mSubsystemManager.setTimeOffset();
      mEnabledLooper.start();
    } catch (Throwable t) {
      Log.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    try {
      Log.logTestInit();
      mEnabledLooper.start();
      mSubsystemManager.checkSystem();
    } catch (Throwable t) {
      Log.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    try {
      mSubsystemManager.slowUpdate();
      mSubsystemManager.outputToSmartDashboard();
      mEnabledLooper.outputToSmartDashboard();
    } catch (Throwable t) {
      Log.logThrowableCrash(t);
      throw t;
    }
  }

}
