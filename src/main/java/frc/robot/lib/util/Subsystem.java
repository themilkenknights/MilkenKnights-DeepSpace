package frc.robot.lib.util;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each
 * subsystem outputs commands to SmartDashboard, has a stop routine (for after each match), and a
 * routine to zero all sensors, which helps with calibration.
 *
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and
 * functions get the instance of the drivetrain and act accordingly. Subsystems are also a state
 * machine with a desired state and actual state; the robot code will try to match the two states
 * with actions. Each Subsystem also is responsible for instantializing all member components at the
 * start of the match.
 */
public abstract class Subsystem {
  // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
  public synchronized void readPeriodicInputs(double timestamp) {
  }

  /**
   * @param timestamp Time in seconds since code start
   */
  public synchronized void onQuickLoop(double timestamp) {
  }

  // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
  public synchronized void writePeriodicOutputs(double timestamp) {
  }

  /**
   * Method for sanity checks with a slow update period
   */
  public synchronized void safetyCheck(double timestamp) {
  }

  /**
   * Method to log data or send to Shuffleboard
   */
  public abstract void outputTelemetry(double timestamp);

  public abstract void teleopInit(double timestamp);

  public abstract void autonomousInit(double timestamp);

  public abstract void onStop(double timestamp);

  public abstract void onRestart(double timestamp);

  public abstract boolean checkSystem();
}
