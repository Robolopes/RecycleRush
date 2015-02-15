
package org.usfirst.frc.team2339.Barracuda;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Talon;


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
    

    /*
    private final RobotDrive robotDrive = 
            new RobotDrive(m_frontLeft, m_frontRight);
    */
    private final SwerveDrive robotDrive = 
            new SwerveDrive();
    private final Talon winchMotor = new Talon(RobotMap.WinchMap.LIFT_WINCH);
      
   
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
    
    /*
     * This method sets winch motors
     * 
     * @param value motor speed
     */
    public void setWinchMotor(double value) {
        SmartDashboard.putNumber("Winch motor value ", value);
        winchMotor.set(value);
    }
    
    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        robotStartTime = System.currentTimeMillis();
        System.out.println("Robot init time: " + robotStartTime);
        
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        
        robotDrive.resetPodAngles();
        
        System.out.println("Before init: " + System.currentTimeMillis());
       //* visionControl.visionInit();
        System.out.println("After init: " + System.currentTimeMillis());
    }
    

    /**
     * This method is called at the beginning of autonomous period
     */
	public void autonomousPeriodicInit() {
		startTime = System.currentTimeMillis();
		System.out.println("Autonomous init time: " + startTime);
       	RobotMap.Control.GYRO.reset();
        robotDrive.resetPodAngles();
        robotDrive.enablePids(true);
    }
	
    public void autonomousPeriodic() {
    }
    
    /**
     * This method is called at the beginning of operator control
     */
    public void teleopInit() {
    	RobotMap.Control.GYRO.reset();
        robotDrive.resetPodAngles();
        robotDrive.enablePids(true);
    }

 

    /**
     * This function is called periodically during operator control
     */

    public void teleopPeriodic() {
   
		/*
		 * Print out siginigicant changes in drive info
		 */
		SmartDashboard.putNumber("Joystick forward ", RobotMap.Control.DRIVE_STICK.getRawAxis(RobotMap.SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK));
		SmartDashboard.putNumber("Joystick sideways ", RobotMap.Control.DRIVE_STICK.getRawAxis(RobotMap.SwerveMap.Control.DRIVE_AXIS_SIDEWAYS));
		SmartDashboard.putNumber("Joystick rotate ", RobotMap.Control.DRIVE_STICK.getRawAxis(RobotMap.SwerveMap.Control.DRIVE_AXIS_ROTATE));
		SmartDashboard.putNumber("Gyro angle ", RobotMap.Control.GYRO.getAngle());
          
    	if(RobotMap.Control.OPERATOR_STICK.getRawButton(RobotMap.Control.GYRO_BUTTON_RESET)) {
    		// Reset gyro
    		RobotMap.Control.GYRO.reset();
    	}
    	
        /*
         * Set winch motors
         */
    	setWinchMotor(RobotMap.WinchMap.WINCH_STICK.getRawAxis(RobotMap.WinchMap.WINCH_AXIS));
    	  
        
		/*
		 * Drive robot based on values from joystick
		 */
		//robotDrive.swerveDriveTeleop();
    	
    	// Test a wheel
    	robotDrive.setWheelPod(SwerveDrive.frontLeft, 180 * RobotMap.Control.DRIVE_STICK.getRawAxis(RobotMap.SwerveMap.Control.DRIVE_AXIS_ROTATE), 
    			RobotMap.Control.DRIVE_STICK.getRawAxis(RobotMap.SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK));
          
    }
 

	/**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        long time = System.currentTimeMillis();
        SmartDashboard.putNumber("Test Mode Running Time ", time);
       // long elapsed = System.currentTimeMillis() - startTime;
        Encoder frontLeftEnc = new Encoder(RobotMap.SwerveMap.DIO.DRIVE_FRONT_LEFT_ENC_A, RobotMap.SwerveMap.DIO.DRIVE_FRONT_LEFT_ENC_A);
        SmartDashboard.putNumber("Front left encoder ", frontLeftEnc.getRaw());
    }
    
}
