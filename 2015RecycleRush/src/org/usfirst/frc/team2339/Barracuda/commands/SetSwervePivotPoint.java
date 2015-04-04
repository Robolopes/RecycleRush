package org.usfirst.frc.team2339.Barracuda.commands;

import org.usfirst.frc.team2339.Barracuda.RobotMap;
import org.usfirst.frc.team2339.Barracuda.components.SwerveJoystick;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDrive;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.RectangularCoordinates;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class SetSwervePivotPoint extends Command {
	
	private boolean isFinished;
	private final SwerveDrive robotDrive;
	private final RectangularCoordinates pivot;
	private final SwerveJoystick driveStick;
	private final double pivotYMin;
	private final double pivotYMax;

	/**
	 * Contruct to set pivot to fixed point
	 * @param name
	 * @param robotDrive
	 * @param pivot
	 */
    public SetSwervePivotPoint(String name, SwerveDrive robotDrive, RectangularCoordinates pivot) {
		super(name);
    	isFinished = false;
    	this.robotDrive = robotDrive;
    	this.pivot = pivot;
    	this.driveStick = null;
    	this.pivotYMin = 0;
    	this.pivotYMax = 0;
    }

    /**
     * Contruct to get pivot from drive stick Z
     * @param name
     * @param robotDrive
     * @param driveStick
     * @param pivotYMin
     * @param pivotYMax
     */
    public SetSwervePivotPoint(String name, SwerveDrive robotDrive, SwerveJoystick driveStick, 
    		double pivotYMin, double pivotYMax) {
		super(name);
    	isFinished = false;
    	this.pivot = new RectangularCoordinates(0, 0);
    	this.robotDrive = robotDrive;
    	this.driveStick = driveStick;
    	this.pivotYMin = pivotYMin;
    	this.pivotYMax = pivotYMax;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	RectangularCoordinates newPivot;
    	if (driveStick != null) {
    		// Set pivot from joystick
	    	double pivotY = pivotYMin + (pivotYMax - pivotYMin) * (driveStick.getPivot() * 0.5 + 0.5);
	    	newPivot = new RectangularCoordinates(0, pivotY);
    	} else {
    		// Set pivot to fixed value
    		newPivot = this.pivot;
    	}
    	SmartDashboard.putNumber("Drive pivot y ", newPivot.y);
    	SmartDashboard.putNumber("Drive pivot front ", newPivot.y - 0.5 * RobotMap.Constants.WHEEL_BASE_LENGTH);
    	robotDrive.setPivot(newPivot);
    	if (driveStick == null) {
    		// Fixed pivot so no need to repeat
    		isFinished = true;
    	}
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
