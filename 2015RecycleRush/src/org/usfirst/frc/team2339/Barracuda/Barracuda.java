
package org.usfirst.frc.team2339.Barracuda;

import org.usfirst.frc.team2339.Barracuda.commands.AutonomousCommand;
import org.usfirst.frc.team2339.Barracuda.commands.AutonomousStrafeLeftCommand;
import org.usfirst.frc.team2339.Barracuda.commands.AutonomousStrafeRightCommand;
import org.usfirst.frc.team2339.Barracuda.smartdashboard.AutoSettings;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.VelocityPolar;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/**
 * The JVM (Java Virtual Machine) on the roboRio is configured to automatically run this class, 
 * and to call the functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Barracuda extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */

    // Control operator interface
    public static OI oi;

    // Commands
    private SendableChooser autoChooser;
    private Command autonomousCommand;
    
      
    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {

    	/*
    	 * Initialize robot components and subsystems
    	 */
    	RobotMap.init();
    	
        // OI must be constructed after subsystems. If the OI creates Commands 
        //(which it very likely will), subsystems are not guaranteed to be 
        // constructed yet. Thus, their requires() statements may grab null 
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Autonomous command
        autoChooser = new SendableChooser();
        autoChooser.addDefault("Strafe Left", new AutonomousStrafeLeftCommand( 
        		RobotMap.Subsystem.robotDrive, RobotMap.Subsystem.lift));
        autoChooser.addObject("Strafe Left", new AutonomousStrafeRightCommand( 
        		RobotMap.Subsystem.robotDrive, RobotMap.Subsystem.lift));
        
    }
    

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){
    	// Zero wheels
       	RobotMap.Subsystem.robotDrive.swerveDriveRobot(new VelocityPolar(0.0, 0.0));
    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This method is called at the beginning of autonomous period
     */
    public void autonomousInit() {
       	RobotMap.Analog.GYRO.reset();
       	//RobotMap.Subsystem.robotDrive.resetSteering();
       	RobotMap.Subsystem.robotDrive.enableSteering(true);
        
        // schedule the autonomous command (example)
        autonomousCommand = (Command)autoChooser.getSelected();
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This method is called at the beginning of operator control
     */
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        //if (autonomousCommand != null) autonomousCommand.cancel();
        
    	RobotMap.Analog.GYRO.reset();
    	//RobotMap.Subsystem.robotDrive.resetSteering();
    	RobotMap.Subsystem.robotDrive.enableSteering(true);
        oi.getTeleopDrive().start();
        oi.getTeleopLift().start();
    }

 

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
    
}
