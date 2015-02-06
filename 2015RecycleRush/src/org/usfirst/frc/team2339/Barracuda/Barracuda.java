
package org.usfirst.frc.team2339.Barracuda;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
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
    

	   /*
     * Initialize joystck variables.
     * The (1) and (2) refer hardward channels 1 and 2 on the robot
     */
    private final Joystick driveStickLeft = new Joystick(0);
    // Only need this if using Tank drive
    private final Joystick driveStickRight = new Joystick(1);
    // Third joystick for shooter and lift control
    private final Joystick operatorStick = new Joystick(2);

    private final Talon m_frontLeft = new Talon(8);
    private final Talon m_frontRight= new Talon(1);
    private final Talon shooterMotorA = new Talon(2);
    private final Talon shooterMotorB = new Talon(3);
    private final Joystick shooterJoystick = operatorStick;
    private final int shooterWinchMotorLoadButton = 7;
    DigitalInput shooterStopSwitch = new DigitalInput(2);
    private boolean currentLoaderMode = false;
    private long shootButtonTime = 0;
    
      private final RobotDrive robotDrive = 
              new RobotDrive(m_frontLeft, m_frontRight);
      
   
      /*
       * This method sets shooter winch motors
       * 
       * @param value motor speed
       */
      public void setShootWinchMotors(double value) {
      //    shooterMotorA.set(value);
      //    shooterMotorB.set(value);
          SmartDashboard.putNumber("Shoot motor value ", value);
      }
      
       

    /*
     * Initialize shooter motor data
     * Numbers are control channels of shooter motors
     */
    
    /*
     * Vision class
    */
   //* private final RecycleRushV2ision visionControl = new RecycleRushVision();

    
    /*
     * Initialize values for autonomous control
     */
    private long startTime = 0;
    private boolean haveShot = false;
    private boolean haveImage = false;
    
    /*
     * Time variables to help with timed printouts
     */
    private long robotStartTime = 0;
    private long time0 = 0;
    
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
        
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        System.out.println("Before init: " + System.currentTimeMillis());
       //* visionControl.visionInit();
        System.out.println("After init: " + System.currentTimeMillis());
    }
    

	public void autonomousPeriodicInit() {
    	   startTime = System.currentTimeMillis();
           System.out.println("Autonomous init time: " + startTime);
 
       }
    public void autonomousPeriodic() {
     
        /*
         * This method is called at the beginning of operator control
         */
    }
     public void teleopInit() {
       
            
        }

 

    /**
     * This function is called periodically during operator control
     */

    public void teleopPeriodic() {
   
          
          /*
           * Get drive data from joystick
           * Tank drive
           */
          double throttleLeft = driveStickLeft.getRawAxis(1);
          double throttleRight = driveStickRight.getRawAxis(1);
          
          
          /*
           * Set shooter winch motors
           */
          if(shooterJoystick.getRawButton(shooterWinchMotorLoadButton) && shooterStopSwitch.get()) {
              // Set shooter motors to load
              setShootWinchMotors(1.0);
          } else {
              // Turn off shooter winch motors
              setShootWinchMotors(0.0);
          }
          
          /*
           * Print out siginigicant changes in drive info
           */
          SmartDashboard.putNumber("Throttle Left ", throttleLeft);
          SmartDashboard.putNumber("Throttle Right ", throttleRight);
          
        
          /*
           * Drive robot based on values from joystick
           */
          robotDrive.tankDrive(-throttleLeft, -throttleRight);
    }
          /*
           * Set shooter winch motors
           */
    
 

	/**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        long time = System.currentTimeMillis();
        SmartDashboard.putNumber("Test Mode Running Time ", time);
       // long elapsed = System.currentTimeMillis() - startTime;

    }
    
}
