/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.lib.geometry.Pose2d;
import frc.robot.lib.structure.Looper;
import frc.robot.lib.util.CrashTracker;
import frc.robot.paths.RobotState;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Superstructure;

import java.util.Arrays;

public class Robot extends TimedRobot {
    public static MatchState mMatchState = MatchState.DISABLED;
    private final SubsystemManager mSubsystemManager = new SubsystemManager(Arrays.asList(Drive.getInstance(), Superstructure.getInstance()));
    private Looper mEnabledLooper = new Looper();

    public Robot() {
        CrashTracker.logRobotConstruction();
    }

    @Override
    public void robotInit() {
        try {
            CrashTracker.logRobotInit();
            mMatchState = MatchState.DISABLED;
            mSubsystemManager.registerEnabledLoops(mEnabledLooper);
            TrajectoryGenerator.getInstance().generateTrajectories();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void disabledInit() {
        try {
            CrashTracker.logDisabledInit();
            mEnabledLooper.stop();
            AutoChooser.disableAuto();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mMatchState = MatchState.DISABLED;
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousInit() {
        try {
            Shuffleboard.startRecording();
            CrashTracker.logAutoInit();
            RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
            mMatchState = MatchState.AUTO;
            mEnabledLooper.start();
            AutoChooser.startAuto();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void teleopInit() {
        try {
            AutoChooser.disableAuto();
            mMatchState = MatchState.TELEOP;
            Shuffleboard.startRecording();
            CrashTracker.logTeleopInit();
            mEnabledLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void testInit() {
        try {
            mMatchState = MatchState.TEST;
            mEnabledLooper.start();
            System.out.println("Starting check systems.");
            mEnabledLooper.stop();
            Drive.getInstance().checkSystem();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void robotPeriodic() {
        try {
            mSubsystemManager.outputToSmartDashboard();
            mEnabledLooper.outputToSmartDashboard();
            RobotState.getInstance().outputToSmartDashboard();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
        Input.updateDriveInput();
    }

    @Override
    public void testPeriodic() {
    }

    public enum MatchState {
        AUTO, TELEOP, DISABLED, TEST
    }
}
