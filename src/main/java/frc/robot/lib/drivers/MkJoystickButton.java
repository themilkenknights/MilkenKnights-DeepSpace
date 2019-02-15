package frc.robot.lib.drivers;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.lib.util.Logger;

public class MkJoystickButton {

	private final MkJoystick joystick;
	private final int rawButton;
	private final String buttonName;
	private boolean lastState;

	public MkJoystickButton(final MkJoystick joystick, final int rawButton, final String buttonName) {
		this.joystick = joystick;
		this.rawButton = rawButton;
		this.buttonName = buttonName;
	}

	/**
	 * Update the last state of the button.
	 */
	private void update() {
		lastState = joystick.getRawButton(rawButton);
	}

	/**
	 * Returns true if the button is pressed and this is the first time it is being run.
	 */
	public boolean isPressed() {
		final boolean isPressed = !lastState && joystick.getRawButton(rawButton);
		update();
		if (isPressed) {
			Logger.logMarker(
					"[Joystick] " + joystick.getPort() + "\t [Button]" + rawButton + "\t Pressed ( "
							+ buttonName + " Button )");
		}
		return isPressed;
	}

	/**
	 * Returns true when the button is released.
	 */
	public boolean isReleased() {
		final boolean isPressed = lastState && joystick.getRawButton(rawButton);
		update();
		return isPressed;
	}

	/**
	 * Returns true when the button is being held.
	 */
	public boolean isHeld() {
		update();
		return joystick.getRawButton(rawButton);
	}

	public int getRawButton() {
		return rawButton;
	}

	public Joystick getJoystick() {
		return joystick;
	}

	public boolean isJoystickConnected() {
		return joystick.getButtonCount() > 0;
	}
}
