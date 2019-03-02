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
import frc.robot.lib.util.MkTime;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.CargoArm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;
import java.util.Arrays;

/**
 * OC Comp TODO:
 * - Flash roboRIO to 2019.2
 * - Verify Pigeon IMU Pitch & Yaw Direction
 * - Test the sensor phase of each encoder
 * - Ensure the sensor phase matches the output of each motor
 * - Run each motor in open-loop through Phoenix Tuner
 * - Check each limit switch with Phoenix Tuner Self-test
 * - Ensure USB Logging is enabled and functional
 *
 * Mechanical/Electrical
 * - Ensure Limelight is mounted and plugged in
 * - Ensure 2 USB Webcams are mounted and plugged in
 * - Ensure the Cargo Arm Breakout board is mounted and verified
 * - Ensure Kettering & Cargo Arm Encoder Cables are shielded
 * - Ensure all mechanisms have full range of rotation with no stutter
 * - Tug test all wires
 */
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
        } catch (Throwable t) {
            Logger.logThrowableCrash(t);
            throw t;
        }
    }

    public enum MatchState {
        AUTO, TELEOP, DISABLED, TEST
    }
}
