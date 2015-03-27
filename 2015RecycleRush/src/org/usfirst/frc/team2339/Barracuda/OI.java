/**
 * 
 */
package org.usfirst.frc.team2339.Barracuda;

import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap;
import org.usfirst.frc.team2339.Barracuda.commands.GyroReset;
import org.usfirst.frc.team2339.Barracuda.commands.SetSwervePivotPoint;
import org.usfirst.frc.team2339.Barracuda.commands.TeleopDrive;
import org.usfirst.frc.team2339.Barracuda.commands.TeleopLift;
import org.usfirst.frc.team2339.Barracuda.components.SwerveJoystick;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.RectangularCoordinates;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * 
 * @author emiller
 */
public class OI {

	public static final double DRIVE_STICK_DEAD_BAND = 0.1;
	public static final int DRIVE_BUTTON_SPEED_SHIFT = 1;
	public static final int DRIVE_BUTTON_ABSOLUTE_GYRO_MODE = 2;
	private static final int DRIVE_BUTTON_ROTATE_AROUND_CONTAINER = 8;
	private static final int GYRO_BUTTON_RESET = 1;
    
    private SwerveJoystick joystickDrive;
    private JoystickButton containerPivotButton;
    
    private Joystick joystickOperator;
    private JoystickButton gyroResetButton;
    
    private TeleopDrive teleopDrive;
    private TeleopLift teleopLift;

	/**
	 * 
	 */
	public OI() {
        setJoystickOperator(new Joystick(1));
        setJoystickDrive(new SwerveJoystick(0));
        setTeleopDrive(new TeleopDrive("Teleop drive", RobotMap.robotDrive, getJoystickDrive(), RobotMap.Control.GYRO));
        setTeleopLift(new TeleopLift("Teleop lift", RobotMap.lift));
        
        containerPivotButton = new JoystickButton(getJoystickDrive(), DRIVE_BUTTON_ROTATE_AROUND_CONTAINER);
        containerPivotButton.whenPressed(new SetSwervePivotPoint("Container Pivot", RobotMap.robotDrive, 
        		new RectangularCoordinates(0.0, SwerveMap.Constants.CONTAINER_CENTER_DISTANCE_FORWARD + 0.5 * SwerveMap.Constants.WHEEL_BASE_LENGTH)));
        containerPivotButton.whenReleased(new SetSwervePivotPoint("Container Pivot", RobotMap.robotDrive, 
        		new RectangularCoordinates(0.0, 0.0)));
        
        gyroResetButton = new JoystickButton(getJoystickOperator(), GYRO_BUTTON_RESET);
        gyroResetButton.whenPressed(new GyroReset(RobotMap.Control.GYRO));
        
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

	/**
	 * @return the teleopDrive
	 */
	public TeleopDrive getTeleopDrive() {
		return teleopDrive;
	}

	/**
	 * @param teleopDrive the teleopDrive to set
	 */
	private void setTeleopDrive(TeleopDrive teleopDrive) {
		this.teleopDrive = teleopDrive;
	}

	/**
	 * @return the teleopLift
	 */
	public TeleopLift getTeleopLift() {
		return teleopLift;
	}

	/**
	 * @param teleopLift the teleopLift to set
	 */
	private void setTeleopLift(TeleopLift teleopLift) {
		this.teleopLift = teleopLift;
	}

}
