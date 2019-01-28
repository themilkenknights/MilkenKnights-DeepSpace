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
import frc.robot.lib.structure.SubsystemManager;
import frc.robot.lib.util.Logger;
import frc.robot.paths.RobotState;
import frc.robot.paths.TrajectoryGenerator;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.HatchArm;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Vision;

import java.util.Arrays;

public class Robot extends TimedRobot {
		public static MatchState mMatchState = MatchState.DISABLED;
		private final SubsystemManager mSubsystemManager =
				new SubsystemManager(Arrays.asList(Drive.getInstance(), Vision.getInstance(), HatchArm.getInstance(), Superstructure.getInstance()));

		protected Robot() {
				super(Constants.kLoopDt);
				Logger.logRobotConstruction();
		}

		@Override
		public void robotInit() {
				try {
						Logger.logRobotInit();
						mMatchState = MatchState.DISABLED;
						TrajectoryGenerator.getInstance().generateTrajectories();
						Shuffleboard.startRecording();
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
						Drive.getInstance().zeroSensors();
						RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
				} catch (Throwable t) {
						Logger.logThrowableCrash(t);
						throw t;
				}
		}

		@Override
		public void autonomousInit() {
				try {
						Logger.logAutoInit();
						Drive.getInstance().zeroSensors();
						RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
						mMatchState = MatchState.AUTO;
						mSubsystemManager.start();
						AutoChooser.startAuto();
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
						mSubsystemManager.start();
				} catch (Throwable t) {
						Logger.logThrowableCrash(t);
						throw t;
				}
		}

		@Override
		public void testInit() {
				try {
						mMatchState = MatchState.TEST;
						mSubsystemManager.start();
						System.out.println("Starting check systems.");
						mSubsystemManager.stop();
						Drive.getInstance().checkSystem();
				} catch (Throwable t) {
						Logger.logThrowableCrash(t);
						throw t;
				}
		}

		@Override
		public void robotPeriodic() {
				try {
						mSubsystemManager.outputToSmartDashboard();
						RobotState.getInstance().outputToSmartDashboard();
				} catch (Throwable t) {
						Logger.logThrowableCrash(t);
						throw t;
				}
		}

		@Override
		public void autonomousPeriodic() {
				mSubsystemManager.onLoop();
		}

		@Override
		public void teleopPeriodic() {
				mSubsystemManager.onLoop();
				Input.updateDriveInput();
		}

		@Override
		public void testPeriodic() {
		}

		public enum MatchState {
				AUTO, TELEOP, DISABLED, TEST
		}
}
