package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib.util.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous mode.
 */
public class AutoModeExecutor {

	private AutoModeBase m_auto_mode;
	private Thread m_thread = null;

	public void start() {
		if (m_thread != null) {
			m_thread.start();
		}
	}

	public void stop() {
		if (m_auto_mode != null) {
			m_auto_mode.stop();
		}
		m_thread = null;
	}

	public AutoModeBase getAutoMode() {
		return m_auto_mode;
	}

	private double timestamp_ = 0;
	private double dt_ = 0;
	private int i = 0;

	public void setAutoMode(AutoModeBase new_auto_mode) {
		m_auto_mode = new_auto_mode;
		m_thread = new Thread(new CrashTrackingRunnable() {
			@Override
			public void runCrashTracked() {
				if (m_auto_mode != null) {
					double now = Timer.getFPGATimestamp();
					m_auto_mode.run();
					dt_ = now - timestamp_;
					timestamp_ = now;
					if (i == 10) {
						SmartDashboard.putNumber("Auto Mode Dt", dt_);
						i = 0;
					}
					i++;
				}
			}
		});
	}
}
