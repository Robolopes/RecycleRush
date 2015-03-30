package org.usfirst.frc.team2339.Barracuda.commands;

import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDrive;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.RectangularCoordinates;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class SetSwervePivotPoint extends Command {
	
	private boolean isFinished;
	private final SwerveDrive robotDrive;
	private final RectangularCoordinates pivot;

    public SetSwervePivotPoint(String name, SwerveDrive robotDrive, RectangularCoordinates pivot) {
		super(name);
    	isFinished = false;
    	this.robotDrive = robotDrive;
    	this.pivot = pivot;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	robotDrive.setPivot(pivot);
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
