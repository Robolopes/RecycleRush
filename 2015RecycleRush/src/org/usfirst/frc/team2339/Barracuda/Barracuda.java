
package org.usfirst.frc.team2339.Barracuda;

import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap;
import org.usfirst.frc.team2339.Barracuda.commands.AutonomousCommand;
import org.usfirst.frc.team2339.Barracuda.commands.TeleopDrive;
import org.usfirst.frc.team2339.Barracuda.commands.TeleopLift;
import org.usfirst.frc.team2339.Barracuda.subsystems.Lift;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDriveRectangle;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Barracuda extends IterativeRobot {
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    /**********************
     * CLASS VARIABLES
    /**********************/
    

	// Subsystems
    private SwerveDriveRectangle robotDrive; 
    private Lift lift;
    
    // Control operator interface
    public static OI oi;

    // Commands
    private AutonomousCommand autonomousCommand;
    private TeleopDrive teleopDrive;
    private TeleopLift teleopLift;
    
      
    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        
        robotDrive = new SwerveDriveRectangle(SwerveDriveRectangle.createWheels(
        		SwerveMap.Constants.WHEEL_BASE_LENGTH, 
        		SwerveMap.Constants.WHEEL_BASE_WIDTH, 
        		SwerveMap.Wheel.DRIVE_CONTROLLERS, 
        		SwerveMap.Wheel.STEERING_PID_CONTROLLERS));
        
        robotDrive.resetSteering();
        
        lift = new Lift(RobotMap.WinchMap.LIFT_WINCH);
        
        // OI must be constructed after subsystems. If the OI creates Commands 
        //(which it very likely will), subsystems are not guaranteed to be 
        // constructed yet. Thus, their requires() statements may grab null 
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Autonomous command
        autonomousCommand = new AutonomousCommand(robotDrive, lift);
        
       //* visionControl.visionInit();
        System.out.println("End robot init: " + System.currentTimeMillis());
    }
    

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){

    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This method is called at the beginning of autonomous period
     */
    public void autonomousInit() {
       	RobotMap.Control.GYRO.reset();
        robotDrive.resetSteering();
        robotDrive.enableSteering(true);
        
        // schedule the autonomous command (example)
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
        if (autonomousCommand != null) autonomousCommand.cancel();
        
    	RobotMap.Control.GYRO.reset();
        robotDrive.resetSteering();
        robotDrive.enableSteering(true);
        teleopDrive = new TeleopDrive("Teleop drive", robotDrive, oi.getJoystickDrive(), RobotMap.Control.GYRO);
        teleopDrive.start();
        teleopLift = new TeleopLift("Teleop lift", lift);
        teleopLift.start();
    }

 

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        
    	if(RobotMap.Control.OPERATOR_STICK.getRawButton(RobotMap.Control.GYRO_BUTTON_RESET)) {
    		// Reset gyro
    		RobotMap.Control.GYRO.reset();
    	}
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
    
}
