package org.usfirst.frc.team2339.Barracuda.commands;

import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap;
import org.usfirst.frc.team2339.Barracuda.components.SwerveJoystick;
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
	private final SwerveJoystick driveStick;

	protected static final double DRIVE_STICK_DEAD_BAND = 0.1;
	protected static final int DRIVE_BUTTON_SPEED_SHIFT = 1;
	protected static final int DRIVE_BUTTON_ROTATE_AROUND_CONTAINER = 8;
	
	/**
	 * 
	 * @param name Name of command
	 * @param robotDrive Robot drive subsystem
	 * @param driveStick joystick used to drive robot in teleop
	 */
	public TeleopDrive(String name, SwerveDrive robotDrive, SwerveJoystick driveStick) {
		super(name);
        requires(robotDrive);
        this.robotDrive = robotDrive;
        this.driveStick = driveStick;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
    	if (driveStick.isInDeadband()) {
    		// Joystick in dead band, set neutral values
    		robotDrive.setDeadBandValues();
    	} else {
    		
        	RobotMotion robotMotion = new RobotMotion(
        			driveStick.getStrafe(), 
        			driveStick.getFrontBack(), 
        			driveStick.getRotate());
            
    		robotMotion.rotate *= .5;

    		RectangularCoordinates pivot = new RectangularCoordinates(0, 0);
            if (driveStick.getRotateAroundContainer()) {
            	pivot.x = 0.0;
            	pivot.y = SwerveMap.Constants.CONTAINER_CENTER_DISTANCE_FORWARD + 0.5 * SwerveMap.Constants.WHEEL_BASE_LENGTH;
            }
            if (driveStick.getSpeedShift()) {
        		robotMotion.rotate *= .5;
        		robotMotion.strafe *= .5;
        		robotMotion.frontBack *= .5;
        	}
            
            double robotAngle = 0.0;
            if (driveStick.getGyro()) {
                robotAngle = SwerveMap.Control.GYRO.getAngle();
            }
            
            robotDrive.swerveDriveAbsolute(robotMotion, robotAngle, pivot);
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
