package frc.robot.lib.util;

import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't roll over
 */
public class CrashTracker {
		private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

		public static void logRobotStartup() {
				logMarker("robot startup");
		}

		public static void logMarker(String mark) {
				logMarker(mark, null);
				System.out.println(mark);
		}

		private static void logMarker(String mark, Throwable nullableException) {
				try (PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt", true))) {
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

		public static void logRobotConstruction() {
				logMarker("robot startup");
		}

		public static void logRobotInit() {
				logMarker("robot init");
		}

		public static void logTeleopInit() {
			logMarker("teleop init");
			Shuffleboard.addEventMarker("Teleop Init", EventImportance.kHigh);
		}

		public static void logAutoInit() {
			logMarker("auto init");
			Shuffleboard.addEventMarker("Auto Init", EventImportance.kHigh);
		}

		public static void logDisabledInit() {
			logMarker("disabled init");
			Shuffleboard.addEventMarker("Disabled Init", EventImportance.kHigh);
		}

		public static void logThrowableCrash(Throwable throwable) {
				logMarker("Exception", throwable);
		}
}
