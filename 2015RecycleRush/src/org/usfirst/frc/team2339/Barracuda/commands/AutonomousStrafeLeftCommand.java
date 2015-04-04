package org.usfirst.frc.team2339.Barracuda.commands;

import org.usfirst.frc.team2339.Barracuda.subsystems.Lift;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDriveRectangle;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Commands to run in autonomous mode
 */
public class AutonomousStrafeLeftCommand extends CommandGroup {
    
    public  AutonomousStrafeLeftCommand(SwerveDriveRectangle robotDrive, Lift lift) {
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
    	
    	addSequential(new TimedLift("Pick up RC", lift, 1, 0.5)); 
    	addSequential(new TimedDrive("Strafe left to auto zone", robotDrive, 2.25, 0.5, 90.0)); 
    }
}
