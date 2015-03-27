/**
 * 
 */
package org.usfirst.frc.team2339.Barracuda.components;

import org.usfirst.frc.team2339.Barracuda.OI;

import edu.wpi.first.wpilibj.Joystick;

/**
 * @author emiller
 *
 */
public class SwerveJoystick extends Joystick {

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
		return  Math.abs(getStrafe()) < OI.DRIVE_STICK_DEAD_BAND && 
    			Math.abs(getFrontBack()) < OI.DRIVE_STICK_DEAD_BAND && 
    			Math.abs(getRotate()) < OI.DRIVE_STICK_DEAD_BAND;
	}
    			
	public boolean getSpeedShift() {
		return getRawButton(OI.DRIVE_BUTTON_SPEED_SHIFT);
	}

	public boolean getGyro() {
		return getRawButton(OI.DRIVE_BUTTON_ABSOLUTE_GYRO_MODE);
	}

}
