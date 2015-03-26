
package org.usfirst.frc.team2339.Barracuda;

import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap;
import org.usfirst.frc.team2339.Barracuda.commands.TimedDrive;
import org.usfirst.frc.team2339.Barracuda.commands.TimedLift;
import org.usfirst.frc.team2339.Barracuda.subsystems.Lift;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDriveRectangle;
import org.usfirst.frc.team2339.Barracuda.commands.TeleopDrive;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.CommandGroup;
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
    

    private SwerveDriveRectangle robotDrive = null; 
    private Lift lift = null;
    private CommandGroup autoCommands = null;
    private TeleopDrive teleopDrive = null;
    
      
   
    /*
     * Vision class
    */
   //* private final RecycleRushV2ision visionControl = new RecycleRushVision();

    
    /*
     * Initialize values for autonomous control
     */
    private long startTime = 0;
    
    /*
     * Time variables to help with timed printouts
     */
    private long robotStartTime = 0;
    
    /**********************
     * CLASS METHODS
     * Methods for this class are below here
    /**********************/
    
    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        robotStartTime = System.currentTimeMillis();
        System.out.println("Robot init time: " + robotStartTime);
        
        robotDrive = new SwerveDriveRectangle(SwerveDriveRectangle.createWheels(
        		SwerveMap.Constants.WHEEL_BASE_LENGTH, 
        		SwerveMap.Constants.WHEEL_BASE_WIDTH, 
        		SwerveMap.Wheel.DRIVE_CONTROLLERS, 
        		SwerveMap.Wheel.STEERING_PID_CONTROLLERS));
        
        robotDrive.resetSteering();
        
        lift = new Lift(RobotMap.WinchMap.LIFT_WINCH);
        
       //* visionControl.visionInit();
        System.out.println("End robot init: " + System.currentTimeMillis());
    }
    

    /**
     * This method is called at the beginning of autonomous period
     */
	public void autonomousPeriodicInit() {
		startTime = System.currentTimeMillis();
		System.out.println("Autonomous init time: " + startTime);
       	RobotMap.Control.GYRO.reset();
        robotDrive.resetSteering();
        robotDrive.enableSteering(true);
        
        autoCommands = new CommandGroup("Autonomous Commands");
        autoCommands.addSequential(new TimedDrive("Push RC to wall", robotDrive, 1.0, 0.5, 0.0));
        autoCommands.addSequential(new TimedLift("Pick up RC", lift, 0.5, 0.25));
        autoCommands.addSequential(new TimedDrive("Backup to auto zone", robotDrive, 2.0, 0.5, 180.0));
        autoCommands.start();
    }
	
    public void autonomousPeriodic() {
    }
    
    /**
     * This method is called at the beginning of operator control
     */
    public void teleopInit() {
    	RobotMap.Control.GYRO.reset();
        robotDrive.resetSteering();
        robotDrive.enableSteering(true);
        teleopDrive = new TeleopDrive("Teleop drive", robotDrive);
        teleopDrive.start();
    }

 

    /**
     * This function is called periodically during operator control
     */

    public void teleopPeriodic() {
   
		/*
		 * Print out siginigicant changes in drive info
		 */
		SmartDashboard.putNumber("Joystick forward ", RobotMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK));
		SmartDashboard.putNumber("Joystick sideways ", RobotMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_SIDEWAYS));
		SmartDashboard.putNumber("Joystick rotate ", RobotMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE));
		SmartDashboard.putNumber("Gyro angle ", RobotMap.Control.GYRO.getAngle());
          
    	if(RobotMap.Control.OPERATOR_STICK.getRawButton(RobotMap.Control.GYRO_BUTTON_RESET)) {
    		// Reset gyro
    		RobotMap.Control.GYRO.reset();
    	}
    	
        /*
         * Set lift motor
         */
    	lift.setLiftMotor(RobotMap.WinchMap.WINCH_STICK.getRawAxis(RobotMap.WinchMap.WINCH_AXIS));
    	  
    	// Test a wheel
    	/*
    	robotDrive.setWheelPod(SwerveDrive.frontLeft, 180 * RobotMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE), 
    			RobotMap.Control.DRIVE_STICK.getRawAxis(RobotMap.SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK));
    	robotDrive.setWheelPod(SwerveDrive.frontRight, 180 * RobotMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE), 
    			RobotMap.Control.DRIVE_STICK.getRawAxis(RobotMap.SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK));
    	robotDrive.setWheelPod(SwerveDrive.rearLeft, 180 * RobotMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE), 
    			RobotMap.Control.DRIVE_STICK.getRawAxis(RobotMap.SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK));
    	robotDrive.setWheelPod(SwerveDrive.rearRight, 180 * RobotMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE), 
    			RobotMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK));
    			*/
          
    }
 

	/**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        long time = System.currentTimeMillis();
        SmartDashboard.putNumber("Test Mode Running Time ", time);
       // long elapsed = System.currentTimeMillis() - startTime;
    }
    
}
