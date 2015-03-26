package org.usfirst.frc.team2339.Barracuda;

import org.usfirst.frc.team2339.Barracuda.components.SwerveSteeringEncoder;
import org.usfirst.frc.team2339.Barracuda.components.SwerveSteeringPidController;
import org.usfirst.frc.team2339.Barracuda.subsystems.Lift;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveDriveRectangle;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Talon;

public class RobotMap {
	
	// Subsystems
    public static SwerveDriveRectangle robotDrive; 
    public static Lift lift;
    
	
	public static class Control {
		// Robot gyro
		public static final Gyro GYRO = new Gyro(0);
		public static final int GYRO_BUTTON_RESET = 1;
		// Joystick for driving
		public static final Joystick DRIVE_STICK = new Joystick(0);
		// Joystick for lift control
		public static final Joystick OPERATOR_STICK = new Joystick(1);
	};
	// put this here to make a diffrence..
	public static class SwerveMap {
		public static class Constants {
			
			// These should be set to actual robot dimensions. 
			// I don't think the units matter but looks like the original is in inches
			public static final double WHEEL_BASE_LENGTH = 28.5;
			public static final double WHEEL_BASE_WIDTH = 25.0;
			// Distance of center of container in front of front wheels. (Negative if behind front wheel line.)
			public static final double CONTAINER_CENTER_DISTANCE_FORWARD = 2.0;
			
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
		
		public static class Control {
			public static final Gyro GYRO = RobotMap.Control.GYRO;
			public static final Joystick DRIVE_STICK = RobotMap.Control.DRIVE_STICK;
			public static final int DRIVE_AXIS_FORWARD_BACK = 1;
			public static final int DRIVE_AXIS_SIDEWAYS = 0;
			public static final int DRIVE_AXIS_ROTATE = 3;
			public static final int DRIVE_BUTTON_ROTATE_AROUND_CONTAINER = 8;
			public static final int DRIVE_BUTTON_ABSOLUTE_GYRO_MODE = 2;
			public static final int DRIVE_BUTTON_SPEED_SHIFT = 1;
			public static final double DRIVE_STICK_DEAD_BAND = 0.1;
		}

		public static class PWM {
			public static final int DRIVE_CONTROLLER[] = {0, 1, 2, 3};
			public static final int STEERING_CONTROLLER[] = {4, 5, 6, 7};
			public static final int DRIVE_FRONT_LEFT = 0;
			public static final int DRIVE_FRONT_LEFT_STEERING = 4;
			public static final int DRIVE_FRONT_RIGHT = 1;
			public static final int DRIVE_FRONT_RIGHT_STEERING = 5;
			public static final int DRIVE_REAR_LEFT = 2;
			public static final int DRIVE_REAR_LEFT_STEERING = 6;
			public static final int DRIVE_REAR_RIGHT = 3;
			public static final int DRIVE_REAR_RIGHT_STEERING = 7;
		};
		
		public static class DIO {
			/*
			 * Swapped values because of this note from AndyMark
			 * Note: Because the encoder is connected to the back of the motor shaft, the rotation is 
			 * backwards from normal front mounted encoders so you will need compensate for this in software or 
			 * simply connect Ch. A output to your Ch. B input and Ch. B output to your Ch. A input.
			 */
			public static final int STEERING_ENCODER_A[] = {0, 2, 4, 6};
			public static final int STEERING_ENCODER_B[] = {1, 3, 5, 7};
			public static final int DRIVE_FRONT_LEFT_ENC_A = 0;
			public static final int DRIVE_FRONT_LEFT_ENC_B = 1;
			public static final int DRIVE_FRONT_RIGHT_ENC_A = 2;
			public static final int DRIVE_FRONT_RIGHT_ENC_B = 3;
			public static final int DRIVE_REAR_LEFT_ENC_A = 4;
			public static final int DRIVE_REAR_LEFT_ENC_B = 5;
			public static final int DRIVE_REAR_RIGHT_ENC_A = 6;
			public static final int DRIVE_REAR_RIGHT_ENC_B = 7;
		};
		
		public static class Wheel {
			public static final int NUMBER_OF_WHEELS = 4;
			
			/*
			 * Controllers are stored in wheel order. Wheels are ordered counter-clockwise
			 * as viewed from top of robot. This agrees with wheel order as defined by SwerveDrive.
			 * For four wheels front right is the first, or wheel number zero. 
			 */
			public static SwerveSteeringPidController STEERING_PID_CONTROLLERS[] = 
					new SwerveSteeringPidController[NUMBER_OF_WHEELS];
			public static Talon DRIVE_CONTROLLERS[] = new Talon[NUMBER_OF_WHEELS];
		}
		
		public static class Analog {
			public static final int GYRO_CHANNEL = 0;
		};

	}
	
	public static class WinchMap {
		public static final Joystick WINCH_STICK = RobotMap.Control.OPERATOR_STICK;
		public static final int WINCH_AXIS = 1;
		public static final int LIFT_WINCH = 8;
	}

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
    			SwerveMap.Constants.STEERING_PID_P, 
    			SwerveMap.Constants.STEERING_PID_I, 
    			SwerveMap.Constants.STEERING_PID_D, 
    			new SwerveSteeringEncoder(steeringEncoderChannelA, 
    					steeringEncoderChannelA, 
    					SwerveMap.Constants.STEERING_ENC_DEGREES_PER_PULSE), 
    			new Talon(steeringMotorControllerPwm));
    }

	/**
	 * Initialize subsystems and components based on RobotMap values
	 */
    public static void init() {
    	
    	/*
    	 * Initialize wheel steering controllers
    	 */
    	for (int iiWheel = 0; iiWheel < SwerveMap.Wheel.NUMBER_OF_WHEELS; iiWheel++) {
        	SwerveMap.Wheel.STEERING_PID_CONTROLLERS[iiWheel] = newSwerveSteeringController(
        			SwerveMap.DIO.STEERING_ENCODER_A[iiWheel],
        			SwerveMap.DIO.STEERING_ENCODER_B[iiWheel],
        			SwerveMap.PWM.STEERING_CONTROLLER[iiWheel]);
    	}
    	
    	/*
    	 * Initialize wheel drive controllers
    	 */
    	for (int iiWheel = 0; iiWheel < SwerveMap.Wheel.NUMBER_OF_WHEELS; iiWheel++) {
    		SwerveMap.Wheel.DRIVE_CONTROLLERS[iiWheel] = new Talon(SwerveMap.PWM.DRIVE_CONTROLLER[iiWheel]);
    	}
    	
    	/*
    	 * Initialize robot drive subsystem
    	 */
        robotDrive = new SwerveDriveRectangle(SwerveDriveRectangle.createWheels(
        		SwerveMap.Constants.WHEEL_BASE_LENGTH, 
        		SwerveMap.Constants.WHEEL_BASE_WIDTH, 
        		SwerveMap.Wheel.DRIVE_CONTROLLERS, 
        		SwerveMap.Wheel.STEERING_PID_CONTROLLERS));
        robotDrive.resetSteering();

        /*
         * Initialize lift subsystem
         */
        lift = new Lift(RobotMap.WinchMap.LIFT_WINCH);
        
    }

}
