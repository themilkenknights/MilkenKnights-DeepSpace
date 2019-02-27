/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.lib.util.Logger;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;
import java.util.Arrays;

public class Robot extends TimedRobot {

    public static MatchState mMatchState = MatchState.DISABLED;
    private final SubsystemManager mSubsystemManager =
        new SubsystemManager(Arrays.asList(Drive.getInstance(), HatchArm.getInstance(), CargoArm.getInstance(), Superstructure.getInstance(), Vision.getInstance()));

    public Robot() {
        super(Constants.GENERAL.kMainLoopDt);
        Logger.logRobotConstruction();
    }

    @Override public void robotInit() {
        try {
            Logger.logRobotInit();
            TrajectoryGenerator.getInstance().generateTrajectories();
            Shuffleboard.startRecording();
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override public void disabledInit() {
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

    @Override public void autonomousInit() {
        try {
            Logger.logAutoInit();
            mMatchState = MatchState.AUTO;
            mSubsystemManager.startAuto();
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override public void teleopInit() {
        try {
            Logger.logTeleopInit();
            mMatchState = MatchState.TELEOP;
            mSubsystemManager.startTeleop();
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    @Override public void testInit() {
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

    @Override public void robotPeriodic() {
        try {
            mSubsystemManager.perioidicUpdate();
            //System.out.println(Drive.getInstance().getPitch());
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    public enum MatchState {
        AUTO, TELEOP, DISABLED, TEST
    }
}
