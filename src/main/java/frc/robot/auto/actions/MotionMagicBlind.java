package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.lib.util.DriveSignal;
import frc.robot.subsystems.Drive;

public class MotionMagicBlind implements Action {

	private double leftDist;
	private double rightDist;

	public MotionMagicBlind(double leftDist, double rightDist) {
		this.leftDist = leftDist;
		this.rightDist = rightDist;
	}

	@Override
	public boolean isFinished() {
		return Drive.getInstance().isMotionMagicFinished();
	}

	@Override
	public void update() {

	}

	@Override
	public void done() {

	}

	@Override
	public void start() {
		Drive.getInstance().setMotionMagicDeltaSetpoint(new DriveSignal(leftDist, rightDist, NeutralMode.Brake), DriveSignal.BRAKE);
	}
}
