/**
 * 
 */
package org.usfirst.frc.team2339.Barracuda.components;

import edu.wpi.first.wpilibj.Joystick;

/**
 * @author emiller
 *
 */
public class OperatorJoystick extends Joystick {

	/**
	 * @param port
	 */
	public OperatorJoystick(int port) {
		super(port);
	}

	/**
	 * @param port
	 * @param numAxisTypes
	 * @param numButtonTypes
	 */
	public OperatorJoystick(int port, int numAxisTypes, int numButtonTypes) {
		super(port, numAxisTypes, numButtonTypes);
	}
	
	public double getLift() {
		return getY();
	}

}
