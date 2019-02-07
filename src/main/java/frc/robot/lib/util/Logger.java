package frc.robot.lib.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't roll over
 */
public class Logger {

	private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

	public static void logRobotConstruction() {
		logMarker("Robot Startup");
	}

	public static synchronized void logMarker(String mark) {
		logMarker(mark, null);
		System.out.println(mark);
	}

	public static void logRobotInit() {
		String dateStamp1 = new SimpleDateFormat("yyyy-MM-dd").format(new Date());
		boolean test = new File("/media/sda1/" + dateStamp1 + "/").mkdirs();
		logMarker("Robot Init");
	}

	public static void logTeleopInit() {
		logMarker("Teleop Init");
		Shuffleboard.addEventMarker("Teleop Init", EventImportance.kHigh);
	}

	public static void logAutoInit() {
		logMarker("Auto Init");
		Shuffleboard.addEventMarker("Auto Init", EventImportance.kHigh);
	}

	public static void logDisabledInit() {
		logMarker("Disabled Init");
		Shuffleboard.addEventMarker("Disabled Init", EventImportance.kHigh);
	}

	public static synchronized void logError(String mark) {
		logMarker(mark, null);
		DriverStation.reportError(mark, false);
	}

	public static synchronized void logCriticalError(String mark) {
		logMarker(mark, null);
		DriverStation.reportError(mark, true);
	}

	public static synchronized void logThrowableCrash(Throwable throwable) {
		logCrashMarker("Exception", throwable);
	}

	private static synchronized void logMarker(String mark, Throwable nullableException) {
		String dateStamp1 = new SimpleDateFormat("yyyy-MM-dd").format(new Date());
		try (PrintWriter writer = new PrintWriter(new FileWriter("/media/sda1/" + dateStamp1 + "/main_log.txt", true))) {
			writer.print(RUN_INSTANCE_UUID.toString());
			writer.print(", ");
			writer.print(mark);
			writer.print(", ");
			writer.print(new Date().toString());
			if (nullableException != null) {
				writer.print(", ");
				nullableException.printStackTrace(writer);
			}
			writer.println();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	private static synchronized void logCrashMarker(String mark, Throwable nullableException) {
		String dateStamp1 = new SimpleDateFormat("yyyy-MM-dd").format(new Date());
		try (PrintWriter writer = new PrintWriter(new FileWriter("/media/sda1/" + dateStamp1 + "/crash_log.txt", true))) {
			writer.print(RUN_INSTANCE_UUID.toString());
			writer.print(", ");
			writer.print(mark);
			writer.print(", ");
			writer.print(new Date().toString());
			if (nullableException != null) {
				writer.print(", ");
				nullableException.printStackTrace(writer);
			}
			writer.println();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
