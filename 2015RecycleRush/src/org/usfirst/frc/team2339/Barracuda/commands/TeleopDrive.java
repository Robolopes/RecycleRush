package org.usfirst.frc.team2339.Barracuda.commands;

import org.usfirst.frc.team2339.Barracuda.components.SwerveJoystick;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDrive;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.RobotMotion;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.VelocityPolar;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopDrive extends Command {
	/**
	 * Drives based on joystick input
	 */
	
	private final SwerveDrive robotDrive;
	private final SwerveJoystick driveStick;
	private final Gyro gyro;

	/**
	 * 
	 * @param name Name of command
	 * @param robotDrive Robot drive subsystem
	 * @param driveStick joystick used to drive robot in teleop
	 */
	public TeleopDrive(String name, SwerveDrive robotDrive, SwerveJoystick driveStick, Gyro gyro) {
		super(name);
        requires(robotDrive);
        this.robotDrive = robotDrive;
        this.driveStick = driveStick;
        this.gyro = gyro;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
		/*
		 * Print out significant changes in drive info
		 */
		SmartDashboard.putNumber("Joystick forward ", driveStick.getFrontBack());
		SmartDashboard.putNumber("Joystick sideways ", driveStick.getStrafe());
		SmartDashboard.putNumber("Joystick rotate ", driveStick.getRotate());
		SmartDashboard.putNumber("Gyro angle ", gyro.getAngle());
          
    	if (driveStick.isInDeadband()) {
    		// Joystick in dead band, set neutral values
    		robotDrive.setDeadBandValues();
    	} else {
    		
        	RobotMotion robotMotion = new RobotMotion(
        			driveStick.getStrafe(), 
        			driveStick.getFrontBack(), 
        			driveStick.getRotate());
            
    		robotMotion.rotate *= .5;

            if (driveStick.getSpeedShift()) {
        		robotMotion.rotate *= .5;
        		robotMotion.strafe *= .5;
        		robotMotion.frontBack *= .5;
        	}
            
            double robotAngle = 0.0;
            if (driveStick.getGyro()) {
                robotAngle = gyro.getAngle();
            }
            
            robotDrive.swerveDriveAbsolute(robotMotion, robotAngle);
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
