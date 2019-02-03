package frc.robot.lib.util;

/**
 * Runnable class with reports all uncaught throws to Logger
 */
public abstract class CrashTrackingRunnable implements Runnable {

	@Override
	public final void run() {
		try {
			runCrashTracked();
		} catch (Throwable t) {
			Logger.logThrowableCrash(t);
			throw t;
		}
	}

	public abstract void runCrashTracked();
}
