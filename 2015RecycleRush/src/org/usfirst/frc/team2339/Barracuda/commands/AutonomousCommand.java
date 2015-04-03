package org.usfirst.frc.team2339.Barracuda.commands;

import org.usfirst.frc.team2339.Barracuda.smartdashboard.AutoSettings;
import org.usfirst.frc.team2339.Barracuda.smartdashboard.SendableTimeSpeed;
import org.usfirst.frc.team2339.Barracuda.smartdashboard.SendableTimeVelocity;
import org.usfirst.frc.team2339.Barracuda.subsystems.Lift;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDriveRectangle;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Commands to run in autonomous mode
 */
public class AutonomousCommand extends CommandGroup {
    
    public  AutonomousCommand(AutoSettings autoSettings, 
    		SwerveDriveRectangle robotDrive, Lift lift) {
    	super("Autonomous Commands");
    	
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    	
    	/*
    	SendableTimeVelocity forward = autoSettings.getForward();
    	if (forward != null && forward.getTime() > 0) {
    		addSequential(new TimedDrive("Push RC to wall", robotDrive, 
    				forward.getTime(), 
    				forward.getVelocity().getSpeed(), 
    				forward.getVelocity().getAngle()));
    	}
    	
    	SendableTimeSpeed liftSettings = autoSettings.getLift();
    	if (liftSettings != null && liftSettings.getTime() > 0) {
    		addSequential(new TimedLift("Pick up RC", lift, 
    				liftSettings.getTime(), 
    				liftSettings.getSpeed()));
    	}
    	
    	SendableTimeVelocity back = autoSettings.getBack();
    	if (back != null && back.getTime() > 0) {
    		addSequential(new TimedDrive("Backup to auto zone", robotDrive, 
    				back.getTime(), 
    				-back.getVelocity().getSpeed(), 
    				back.getVelocity().getAngle()));
    	}
    	*/
    	//addSequential(new TimedDrive("Push RC to wall", robotDrive, 1.25, 0.5, 0.0)); 
    	addSequential(new TimedLift("Pick up RC", lift, 1, 0.5)); 
    	//addSequential(new TimedDrive("Backup to auto zone", robotDrive, 1.5, -0.5, 0.0)); 
    	addSequential(new TimedDrive("Strafe left to auto zone", robotDrive, 1.5, 0.5, 90.0)); 
    }
}
