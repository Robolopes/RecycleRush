/**
 * 
 */
package org.usfirst.frc.team2339.Barracuda;

import org.usfirst.frc.team2339.Barracuda.components.SwerveJoystick;

import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * 
 * @author emiller
 */
public class OI {

    private SwerveJoystick joystickDrive;
    private Joystick joystickOperator;
    
	/**
	 * 
	 */
	public OI() {
        setJoystickOperator(new Joystick(1));
        setJoystickDrive(new SwerveJoystick(0));
	}

	/**
	 * @return the joystickDrive
	 */
	public SwerveJoystick getJoystickDrive() {
		return joystickDrive;
	}

	/**
	 * @param joystickDrive the joystickDrive to set
	 */
	protected void setJoystickDrive(SwerveJoystick joystickDrive) {
		this.joystickDrive = joystickDrive;
	}

	/**
	 * @return the joystickOperator
	 */
	public Joystick getJoystickOperator() {
		return joystickOperator;
	}

	/**
	 * @param joystickOperator the joystickOperator to set
	 */
	protected void setJoystickOperator(Joystick joystickOperator) {
		this.joystickOperator = joystickOperator;
	}

}
