package org.usfirst.frc.team2339.Barracuda;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class RobotMap {
	
	public static class Constants {
		public static final double STEERING_ENC_REVOLUTIONS_PER_PULSE = 2;
		public static final double STEERING_PID_P = 1;
		public static final double STEERING_PID_I = 1;
		public static final double STEERING_PID_D = 1;
		
	};
	
	public static class Control {
		 // Joystick for driving
		public static final Joystick DRIVE_STICK = new Joystick(0);
		public static final int DRIVE_AXIS_FORWARD_BACK = 1;
		public static final int DRIVE_AXIS_SIDEWAYS = 2;
		public static final int DRIVE_AXIS_ROTATE = 3;
		public static final int DRIVE_CONTROLLER_SHIFT_LOW = 1;
		public static final int DRIVE_CONTROLLER_SHIFT_HIGH = 2;
		// Joystick for lift control
		public static final Joystick OPERATOR_STICK = new Joystick(1);
	};
	
	public static class Solenoid {
		public static final int DRIVE_SHIFT_HIGH = 1;
		public static final int DRIVE_SHIFT_LOW = 2;
	};
	
	public static class PWM {
		public static final int DRIVE_FRONT_LEFT = 0;
		public static final int DRIVE_FRONT_LEFT_STEERING = 1;
		public static final int DRIVE_FRONT_RIGHT = 2;
		public static final int DRIVE_FRONT_RIGHT_STEERING = 3;
		public static final int DRIVE_REAR_LEFT = 4;
		public static final int DRIVE_REAR_LEFT_STEERING = 5;
		public static final int DRIVE_REAR_RIGHT = 6;
		public static final int DRIVE_REAR_RIGHT_STEERING = 7;
		public static final int LIFT_WINCH = 8;
		
	};
	
	public static class DIO {
		public static final int DRIVE_FRONT_LEFT_ENC_A = 0;
		public static final int DRIVE_FRONT_LEFT_ENC_B = 1;
		public static final int DRIVE_FRONT_RIGHT_ENC_A = 2;
		public static final int DRIVE_FRONT_RIGHT_ENC_B = 3;
		public static final int DRIVE_REAR_LEFT_ENC_A = 4;
		public static final int DRIVE_REAR_LEFT_ENC_B = 5;
		public static final int DRIVE_REAR_RIGHT_ENC_A = 6;
		public static final int DRIVE_REAR_RIGHT_ENC_B = 7;

	};
	
	
	public final int shooterWinchMotorLoadButton = 7;
	public DigitalInput shooterStopSwitch = new DigitalInput(2);
	public boolean currentLoaderMode = false;
	public long shootButtonTime = 0;
}
