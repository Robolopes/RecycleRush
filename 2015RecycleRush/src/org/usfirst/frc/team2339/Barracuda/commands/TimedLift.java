package org.usfirst.frc.team2339.Barracuda.commands;

import org.usfirst.frc.team2339.Barracuda.subsystems.Lift;

import edu.wpi.first.wpilibj.command.Command;

public class TimedLift extends Command {
	/**
	 * Runs lift for a given time and speed
	 */
	
	private final Lift lift;
	private final double time;
	private final double speed;

	/**
	 * 
	 * @param name Name of command
	 * @param lift lift subsystem
	 * @param time Time to drive (in seconds)
	 * @param speed Speed to drive at [0, 1]
	 */
	public TimedLift(String name, Lift lift, 
			double time, double speed) {
		super(name);
        requires(lift);
        this.lift = lift;
		this.time = time;
		this.speed = speed;
	}

	@Override
	protected void initialize() {
	}

	@Override
	protected void execute() {
		lift.setLiftMotor(speed);
	}

	@Override
	protected boolean isFinished() {
		return timeSinceInitialized() >= time;
	}

	@Override
	protected void end() {
		lift.stopLiftMotor();
	}

	@Override
	protected void interrupted() {
		lift.stopLiftMotor();
	}

}
