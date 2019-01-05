package frc.robot.util.logging;

/**
 * Runnable class with reports all uncaught throws to Log
 */
public abstract class CrashTrackingRunnable implements Runnable {

    @Override
    public final void run() {
        try {
            runCrashTracked();
        } catch (Throwable t) {
            Log.logThrowableCrash(t);
            throw t;
        }
    }

    public abstract void runCrashTracked();
}
