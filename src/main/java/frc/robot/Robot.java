/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.lib.structure.Looper;
import frc.robot.lib.util.CrashTracker;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Drive.DriveControlState;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Superstructure;
import java.util.Arrays;

public class Robot extends TimedRobot {

	private final SubsystemManager mSubsystemManager = new SubsystemManager(
			Arrays.asList(Drive.getInstance(), Superstructure.getInstance(), Input.getInstance()));
	private Looper mEnabledLooper = new Looper();

	public Robot() {
		CrashTracker.logRobotStartup();
	}

	@Override
	public void robotInit() {
		try {
			CrashTracker.logRobotInit();
			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
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
			ControlState.resetDefaultState();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void autonomousInit() {
		try {
			CrashTracker.logAutoInit();
			ControlState.mDriveControlState = DriveControlState.VISION_TRACKING;
			mEnabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
		try {
			CrashTracker.logTeleopInit();
			mEnabledLooper.start();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void testInit() {
		try {
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
	public void testPeriodic() {
	}

	@Override
	public void robotPeriodic() {
		try {
			mSubsystemManager.outputToSmartDashboard();
			mEnabledLooper.outputToSmartDashboard();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

}
