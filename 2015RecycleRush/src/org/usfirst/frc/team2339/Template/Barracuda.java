
package org.usfirst.frc.team2339.Template;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
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
     * Smart dashboard preferences
     */
    private final Preferences preferences = Preferences.getInstance();
    private int redLow;
    
	   /*
     * Initialize joystck variables.
     * The (1) and (2) refer hardward channels 1 and 2 on the robot
     */
    private final Joystick driveStickLeft = new Joystick(0);
    // Only need this if using Tank drive
    private final Joystick driveStickRight = new Joystick(1);
    // Third joystick for shooter and lift control
    private final Joystick operatorStick = new Joystick(2);
    private final DriverStation driverStation = DriverStation.getInstance();
    // Class to interact with driver station
    //private final DriverStationEnhancedIO driverStation = DriverStation.getInstance().getEnhancedIO();
    
    /*
     * Initialize motor controllers.
     */
    private final Talon m_frontLeft = new Talon(0);
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
       * Initialize super shifter
       */

      private final int clawButton = 1;
      private final Joystick clawJoystick = operatorStick;
      private boolean isClawClosed = false;
      private boolean wasClawButtonJustPushed = false;
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
     * Initialize compressor
     */
      
    private final int compressorRelayChannel = 1;
    private final Compressor compressor = new Compressor(compressorRelayChannel);
    
    
    /*
     * Initialize relay to control cooling fan
     */
    private final Relay fan = new Relay(2);

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
        
        redLow = preferences.getInt("Red low", 200);
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        System.out.println("Before init: " + System.currentTimeMillis());
       //* visionControl.visionInit();
        System.out.println("After init: " + System.currentTimeMillis());
    }
    
    private void setShootSolenoid(boolean b) {
		// TODO Auto-generated method stub
		
	}

	public void autonomousPeriodicInit() {
    	   startTime = System.currentTimeMillis();
           System.out.println("Autonomous init time: " + startTime);
           // Start compressor
           compressor.start();
           // Turn on cooling fan at beginning of autonomous
           fan.set(Relay.Value.kForward);
           // Initialize drive encoder

           haveShot = false;
           haveImage = false;
       }
    public void autonomousPeriodic() {
        long elapsed = System.currentTimeMillis() - startTime;
        SmartDashboard.putNumber("Autonomous Elapsed time ", elapsed/1000.0);
        double shooterSlider = 5.0;
        double driveTimeSlider = 2.0;
        double driveSpeedSlider = 5.0;
        SmartDashboard.putNumber("Autonomous Shooter Slider ", shooterSlider);
        SmartDashboard.putBoolean("Have shot ", haveShot);
        

        final long delayTime = 200;
        // Start bot about 3 ft behind line.
        final long driveTime = delayTime + 1500; 
        final long drivePauseTime = driveTime + 1000;
  
        /*
         * This method is called at the beginning of operator control
         */
    }
     public void teleopInit() {
            // Start compressor
            compressor.start();
            // Turn on cooling fan at beginning of teleop
            fan.set(Relay.Value.kForward);
            // Shift to high for teleop
            isClawClosed = false;
            
            String operatorButtons = "";
            for (int iiButton = 0; iiButton < 10; iiButton++) {
                if (iiButton > 0) {
                    operatorButtons += " ";
                }
                operatorButtons += (iiButton+1) + ":" + operatorStick.getRawButton(iiButton+1);
            }
            SmartDashboard.putString("Operator Buttons ", operatorButtons);

        }

 

    /**
     * This function is called periodically during operator control
     */

    public void teleopPeriodic() {
    	
          boolean pressure = compressor.getPressureSwitchValue();
          /*
           * Keep fan running
           */
          fan.set(Relay.Value.kForward);
          
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
        long elapsed = System.currentTimeMillis() - startTime;
    	compressor.start();

    }
    
}
