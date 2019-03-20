package frc.robot.lib.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.UUID;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.Constants.MISC;

/**
 * Tracks start-up and caught crash events, logging them to a file which dosn't roll over
 */
public class Logger {
  private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();
  private static String lastOutput = "";
  private static String lastCrashOutput = "";
  private static MkTimer lastMarkerTimer = new MkTimer();
  private static MkTimer lastCrashMarkerTimer = new MkTimer();

  public static void logRobotConstruction() {
    logMarker("Robot Startup");
  }

  public static synchronized void logMarker(String mark) {
    logMarker(mark, null);
    System.out.println(mark);
  }

  private static synchronized void logMarker(String mark, Throwable nullableException) {
    if (MISC.kErrorLogging) {
      if (!lastOutput.equals(mark) || lastMarkerTimer.isDone()) {
        if (lastMarkerTimer.isDone()) {
          lastMarkerTimer.reset();
        }
        lastOutput = mark;
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
      } else {
        lastMarkerTimer.start(0.5);
      }
    }
  }

  public static void logRobotInit() {
    String dateStamp1 = new SimpleDateFormat("yyyy-MM-dd").format(new Date());
    boolean test = new File("/media/sda1/" + dateStamp1 + "/").mkdirs();
    logMarker("Robot Init on " + (Constants.kIsPracticeBot ? "Practice" : "Competition") + " Robot");
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

  public static synchronized void logErrorWithTrace(String mark) {
    logMarker(mark, null);
    DriverStation.reportError(mark, false);
  }

  public static synchronized void logThrowableCrash(Throwable throwable) {
    logCrashMarker("Exception", throwable);
  }

  private static synchronized void logCrashMarker(String mark, Throwable nullableException) {
    if (MISC.kErrorLogging) {
      if (!lastCrashOutput.equals(mark) || lastCrashMarkerTimer.isDone()) {
        if (lastCrashMarkerTimer.isDone()) {
          lastCrashMarkerTimer.reset();
        }
        lastCrashOutput = mark;
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
      } else {
        lastCrashMarkerTimer.start(0.5);
      }
    }
  }
}
