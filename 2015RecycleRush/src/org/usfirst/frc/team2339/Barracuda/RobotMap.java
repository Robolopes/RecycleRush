package org.usfirst.frc.team2339.Barracuda;

import org.usfirst.frc.team2339.Barracuda.components.SwerveSteeringEncoder;
import org.usfirst.frc.team2339.Barracuda.components.SwerveSteeringPidController;
import org.usfirst.frc.team2339.Barracuda.subsystems.Lift;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDriveRectangle;

import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Talon;

public class RobotMap {
	
	public static class Constants {
		
		public static final int NUMBER_OF_WHEELS = 4;
		
		// These should be set to actual robot dimensions. 
		// The units do not matter as long as they are consistent. Numbers below are in inches.
		public static final double WHEEL_BASE_LENGTH = 28.5;
		public static final double WHEEL_BASE_WIDTH = 25.0;
		// Distance of center of container in front of front wheels. (Negative if behind front wheel line.)
		public static final double CONTAINER_CENTER_DISTANCE_FORWARD = 18.0;
		
		/*
		 * AndyMark Gearmotor am-2971. See http://www.andymark.com/product-p/am-2971.htm
		 * This number is encoder pulse per wheel revolution
		 * am-2971 has encoder on motor shaft that gives 7 pulses per motor turn
		 * am-2971 has a 71:1 gear ratio.
		 * 7 pulse per motor revolution * 71 gear ratio ==> 497 pulses per gear box output.
		 * Gear box output has 22 tooth gear, wheel has 32 tooth gear.
		 * 341.69 = 497 * 22/32 pulses per wheel revolution.
		 * 341.69/360 pulses per degree
		 * So degrees per pulse 360/341.69 <== (32 * 360) / ( 7 * 71 * 22)
		 * 2015-02-15: Testing revealed the need to divide by two, thus the extra "2.0"
		 */
		public static final double STEERING_ENC_DEGREES_PER_PULSE = (32.0 * 360.0) / (7.0 * 71.0 * 22.0 * 2.0);
		public static final double STEERING_PID_P = 0.02;
		public static final double STEERING_PID_I = 0;
		public static final double STEERING_PID_D = 0;
	};
	
	
	public static class Analog {
		public static final Gyro GYRO = new Gyro(0);
	};
	
	public static class PWM {
		/*
		 * Drive and steering PWMs.
		 * Values are counter-clockwise viewed from top of robot. First value is front right.
		 * Thus order is front right, front left, rear left, rear right 
		 */
		public static final int DRIVE_CONTROLLERS[] = {1, 0, 2, 3};
		public static final int STEERING_CONTROLLERS[] = {5, 4, 6, 7};
		public static final int LIFT_WINCH = 8;
	};
	
	public static class DIO {
		/*
		 * In theory should swap values because of this note from AndyMark. However, our experience is they are not swapped.
		 * Note: Because the encoder is connected to the back of the motor shaft, the rotation is 
		 * backwards from normal front mounted encoders so you will need compensate for this in software or 
		 * simply connect Ch. A output to your Ch. B input and Ch. B output to your Ch. A input.
		 * 
		 * Order is same as wheel order (see above): front right, front left, rear left, rear right
		 */
		public static final int STEERING_ENCODERS_A[] = {2, 0, 4, 6};
		public static final int STEERING_ENCODERS_B[] = {3, 1, 5, 7};
		public static final int LIFT_LOWER_LIMIT_SWITCH = 8;
	};
	
	public static class Subsystem {
	    public static SwerveDriveRectangle robotDrive; 
	    public static Lift lift;
	};
    
    /**
	 * Create a swerve steering controller
	 * 
     * @param steeringEncoderChannelA First steering encoder DIO channel 
     * @param steeringEncoderChannelB Second steering encoder DIO channel
     * @param steeringMotorControllerPwm Steering motor controller PWM channel
     * @return new swerve steering controller
     */
    public static SwerveSteeringPidController newSwerveSteeringController(
    		int steeringEncoderChannelA, 
    		int steeringEncoderChannelB, 
    		int steeringMotorControllerPwm) {
    	
    	return new SwerveSteeringPidController(
    			Constants.STEERING_PID_P, 
    			Constants.STEERING_PID_I, 
    			Constants.STEERING_PID_D, 
    			new SwerveSteeringEncoder(steeringEncoderChannelA, 
    					steeringEncoderChannelB, 
    					Constants.STEERING_ENC_DEGREES_PER_PULSE), 
    			new Talon(steeringMotorControllerPwm));
    }

	/**
	 * Initialize subsystems and components based on RobotMap values
	 */
    public static void init() {
    	
    	/*
    	 * Initialize wheel steering controllers
    	 */
		SwerveSteeringPidController steeringPidControllers[] = 
				new SwerveSteeringPidController[Constants.NUMBER_OF_WHEELS];
    	for (int iiWheel = 0; iiWheel < Constants.NUMBER_OF_WHEELS; iiWheel++) {
    		steeringPidControllers[iiWheel] = newSwerveSteeringController(
        			DIO.STEERING_ENCODERS_A[iiWheel],
        			DIO.STEERING_ENCODERS_B[iiWheel],
        			PWM.STEERING_CONTROLLERS[iiWheel]);
    	}
    	
    	/*
    	 * Initialize wheel drive controllers
    	 */
		Talon driveControllers[] = new Talon[Constants.NUMBER_OF_WHEELS];
    	for (int iiWheel = 0; iiWheel < Constants.NUMBER_OF_WHEELS; iiWheel++) {
    		driveControllers[iiWheel] = new Talon(PWM.DRIVE_CONTROLLERS[iiWheel]);
    	}
    	
    	/*
    	 * Initialize robot drive subsystem
    	 */
        Subsystem.robotDrive = new SwerveDriveRectangle(SwerveDriveRectangle.createWheels(
        		Constants.WHEEL_BASE_LENGTH, 
        		Constants.WHEEL_BASE_WIDTH, 
        		driveControllers, 
        		steeringPidControllers));
        Subsystem.robotDrive.resetSteering();

        /*
         * Initialize lift subsystem
         */
        Subsystem.lift = new Lift(PWM.LIFT_WINCH, DIO.LIFT_LOWER_LIMIT_SWITCH);
        
    }

}
