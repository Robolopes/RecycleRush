package org.usfirst.frc.team2339.Barracuda.commands;

import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDrive;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.VelocityPolar;

import edu.wpi.first.wpilibj.command.Command;

public class TimedDrive extends Command {
	/**
	 * Drives for a given time in at a given speed and direction
	 */
	
	private final SwerveDrive robotDrive;
	private final double time;
	private final double speed;
	private final double direction;

	/**
	 * 
	 * @param name Name of command
	 * @param robotDrive Robot drive subsystem
	 * @param time Time to drive (in seconds)
	 * @param speed Speed to drive at [0, 1]
	 * @param direction Direction to drive [-180, 180]. Zero is forward.
	 */
	public TimedDrive(String name, SwerveDrive robotDrive, 
			double time, double speed, double direction) {
		super(name);
        requires(robotDrive);
        this.robotDrive = robotDrive;
		this.time = time;
		this.speed = speed;
		this.direction = direction;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
		robotDrive.swerveDriveRobot(new VelocityPolar(speed, direction));
	}

	@Override
	protected boolean isFinished() {
		return timeSinceInitialized() >= time;
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
