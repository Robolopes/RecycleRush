package org.usfirst.frc.team2339.Barracuda.commands;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class GyroReset extends Command {

	private boolean isFinished;
	private final Gyro gyro;
	
    public GyroReset(Gyro gyro) {
    	isFinished = false;
    	this.gyro = gyro;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	gyro.reset();
    	isFinished = true;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return isFinished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
