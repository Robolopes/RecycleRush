package org.usfirst.frc.team2339.Barracuda;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Gyro;

public class RobotMap {
	
	public static class Control {
		// Robot gyro
		public static final Gyro GYRO = new Gyro(0);
		public static final int GYRO_BUTTON_RESET = 1;
		// Joystick for driving
		public static final Joystick DRIVE_STICK = new Joystick(0);
		// Joystick for lift control
		public static final Joystick OPERATOR_STICK = new Joystick(1);
	};
	
	public static class SwerveMap {
		public static class Constants {
			// These should be set to actual robot dimensions. 
			// I don't think the units matter but looks like the original is in inches
			public static final double WHEEL_BASE_LENGTH = 28;
			public static final double WHEEL_BASE_WIDTH = 38;
			/*
			 * AndyMark Gearmotor am-2971. See http://www.andymark.com/product-p/am-2971.htm
			 * This number is encoder pulse per wheel revolution
			 * am-2971 has encoder on motor shaft that gives 7 pulses per motor turn
			 * am-2971 has a 71:1 gear ratio.
			 * 7 pulse per motor revolution * 71 gear ratio ==> 497 pulses per gear box output.
			 * Gear box output has 22 tooth gear, wheel has 32 tooth gear.
			 * 341.69 = 497 * 22/32 pulses per wheel revolution 
			 */
			public static final double STEERING_ENC_PULSES_PER_REVOLUTION = 7.0 * 71.0 * 22.0 / 32.0;
			public static final double STEERING_PID_P = 0;
			public static final double STEERING_PID_I = 0;
			public static final double STEERING_PID_D = 0;
		};
		
		public static class Control {
			public static final Gyro GYRO = RobotMap.Control.GYRO;
			public static final Joystick DRIVE_STICK = RobotMap.Control.DRIVE_STICK;
			public static final int DRIVE_AXIS_FORWARD_BACK = 1;
			public static final int DRIVE_AXIS_SIDEWAYS = 2;
			public static final int DRIVE_AXIS_ROTATE = 3;
			public static final int DRIVE_CONTROLLER_SHIFT_LOW = 1;
			public static final int DRIVE_CONTROLLER_SHIFT_HIGH = 2;
			public static final double DRIVE_STICK_DEAD_BAND = 0.1;
		}

		public static class PWM {
			public static final int DRIVE_FRONT_LEFT = 0;
			public static final int DRIVE_FRONT_LEFT_STEERING = 1;
			public static final int DRIVE_FRONT_RIGHT = 2;
			public static final int DRIVE_FRONT_RIGHT_STEERING = 3;
			public static final int DRIVE_REAR_LEFT = 4;
			public static final int DRIVE_REAR_LEFT_STEERING = 5;
			public static final int DRIVE_REAR_RIGHT = 6;
			public static final int DRIVE_REAR_RIGHT_STEERING = 7;
		};
		
		public static class DIO {
			/*
			 * Swapped values because of this note from AndyMark
			 * Note: Because the encoder is connected to the back of the motor shaft, the rotation is 
			 * backwards from normal front mounted encoders so you will need compensate for this in software or 
			 * simply connect Ch. A output to your Ch. B input and Ch. B output to your Ch. A input.
			 */
			public static final int DRIVE_FRONT_LEFT_ENC_A = 1;
			public static final int DRIVE_FRONT_LEFT_ENC_B = 0;
			public static final int DRIVE_FRONT_RIGHT_ENC_A = 3;
			public static final int DRIVE_FRONT_RIGHT_ENC_B = 2;
			public static final int DRIVE_REAR_LEFT_ENC_A = 5;
			public static final int DRIVE_REAR_LEFT_ENC_B = 4;
			public static final int DRIVE_REAR_RIGHT_ENC_A = 7;
			public static final int DRIVE_REAR_RIGHT_ENC_B = 6;
		};
		
		public static class Analog {
			public static final int GYRO_CHANNEL = 0;
		};

		public static class Solenoid {
			public static final int DRIVE_SHIFT_HIGH = 1;
			public static final int DRIVE_SHIFT_LOW = 2;
		};
	}
	
	public static class WinchMap {
		public static final Joystick WINCH_STICK = RobotMap.Control.OPERATOR_STICK;
		public static final int WINCH_BUTTON_UP = 1;
		public static final int WINCH_BUTTON_DOWN = 2;
		public static final int LIFT_WINCH = 8;
	}
}
