package org.usfirst.frc.team2339.Barracuda.commands;

import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDrive;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.RectangularCoordinates;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.RobotMotion;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.VelocityPolar;

import edu.wpi.first.wpilibj.command.Command;

public class TeleopDrive extends Command {
	/**
	 * Drives based on joystick input
	 */
	
	private final SwerveDrive robotDrive;

	/**
	 * 
	 * @param name Name of command
	 * @param robotDrive Robot drive subsystem
	 * @param time Time to drive (in seconds)
	 * @param speed Speed to drive at [0, 1]
	 * @param direction Direction to drive [-180, 180]. Zero is forward.
	 */
	public TeleopDrive(String name, SwerveDrive robotDrive) {
		super(name);
        requires(robotDrive);
        this.robotDrive = robotDrive;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
    	RobotMotion robotMotion = new RobotMotion(
    			SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_SIDEWAYS), 
    			-SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK), 
    			SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE));
        
    	if (Math.abs(robotMotion.strafe) > SwerveMap.Control.DRIVE_STICK_DEAD_BAND || 
    			Math.abs(robotMotion.frontBack) > SwerveMap.Control.DRIVE_STICK_DEAD_BAND || 
    			Math.abs(robotMotion.rotate) > SwerveMap.Control.DRIVE_STICK_DEAD_BAND) {
    		robotMotion.rotate *= .5;

    		RectangularCoordinates pivot = new RectangularCoordinates(0, 0);
            if (SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_BUTTON_ROTATE_AROUND_CONTAINER)) {
            	pivot.x = 0.0;
            	pivot.y = SwerveMap.Constants.CONTAINER_CENTER_DISTANCE_FORWARD + 0.5 * SwerveMap.Constants.WHEEL_BASE_LENGTH;
            }
            if (SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_BUTTON_SPEED_SHIFT)) {
        		robotMotion.rotate *= .5;
        		robotMotion.strafe *= .5;
        		robotMotion.frontBack *= .5;
        	}
            
            double robotAngle = 0.0;
            if (SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_BUTTON_ABSOLUTE_GYRO_MODE)) {
                robotAngle = SwerveMap.Control.GYRO.getAngle();
            }
            
            robotDrive.swerveDriveAbsolute(robotMotion, robotAngle, pivot);
    	} else {
    		// Joystick in dead band, set neutral values
    		robotDrive.setDeadBandValues();
    	}
	}

	@Override
	protected boolean isFinished() {
		return false;
	}

	@Override
	protected void end() {
		robotDrive.swerveDriveRobot(new VelocityPolar(0.0, 0.0));
	}

	@Override
	protected void interrupted() {
		robotDrive.swerveDriveRobot(new VelocityPolar(0.0, 0.0));
	}

}
