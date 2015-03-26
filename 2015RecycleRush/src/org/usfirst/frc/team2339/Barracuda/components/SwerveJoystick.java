/**
 * 
 */
package org.usfirst.frc.team2339.Barracuda.components;

import edu.wpi.first.wpilibj.Joystick;

/**
 * @author emiller
 *
 */
public class SwerveJoystick extends Joystick {

	protected static final double DRIVE_STICK_DEAD_BAND = 0.1;
	protected static final int DRIVE_BUTTON_SPEED_SHIFT = 1;
	public static final int DRIVE_BUTTON_ABSOLUTE_GYRO_MODE = 2;
	protected static final int DRIVE_BUTTON_ROTATE_AROUND_CONTAINER = 8;
	
	/**
	 * @param port
	 */
	public SwerveJoystick(int port) {
		super(port);
	}

	/**
	 * @param port
	 * @param numAxisTypes
	 * @param numButtonTypes
	 */
	public SwerveJoystick(int port, int numAxisTypes, int numButtonTypes) {
		super(port, numAxisTypes, numButtonTypes);
	}
	
	public double getStrafe() {
		return getX();
	}

	public double getFrontBack() {
		return -getY();
	}

	public double getRotate() {
		return getTwist();
	}

	public boolean isInDeadband() {
		return  Math.abs(getStrafe()) < DRIVE_STICK_DEAD_BAND && 
    			Math.abs(getFrontBack()) < DRIVE_STICK_DEAD_BAND && 
    			Math.abs(getRotate()) < DRIVE_STICK_DEAD_BAND;
	}
    			
	public boolean getSpeedShift() {
		return getRawButton(DRIVE_BUTTON_SPEED_SHIFT);
	}

	public boolean getGyro() {
		return getRawButton(DRIVE_BUTTON_ABSOLUTE_GYRO_MODE);
	}

	public boolean getRotateAroundContainer() {
		return getRawButton(DRIVE_BUTTON_ROTATE_AROUND_CONTAINER);
	}

}
