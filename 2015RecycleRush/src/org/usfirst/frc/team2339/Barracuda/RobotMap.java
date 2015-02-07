package org.usfirst.frc.team2339.Barracuda;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Encoder;

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
		public static final Talon DRIVE_FRONT_LEFT = new Talon(0);
		public static final Talon DRIVE_FRONT_LEFT_STEERING = new Talon(1);
		public static final Talon DRIVE_FRONT_RIGHT = new Talon(2);
		public static final Talon DRIVE_FRONT_RIGHT_STEERING = new Talon(3);
		public static final Talon DRIVE_REAR_LEFT = new Talon(4);
		public static final Talon DRIVE_REAR_LEFT_STEERING = new Talon(5);
		public static final Talon DRIVE_REAR_RIGHT = new Talon(6);
		public static final Talon DRIVE_REAR_RIGHT_STEERING = new Talon(7);
		public static final Talon LIFT_WINCH = new Talon(8);
		
	};
	
	public static class DIO {
		public static final Encoder DRIVE_FRONT_LEFT_ENC = new Encoder(0, 1);
		public static final Encoder DRIVE_FRONT_RIGHT_ENC = new Encoder(2, 3);
		public static final Encoder DRIVE_REAR_LEFT_ENC = new Encoder(4, 5);
		public static final Encoder DRIVE_REAR_RIGHT_ENC = new Encoder(6, 7);

	};
	
	
	public final int shooterWinchMotorLoadButton = 7;
	public DigitalInput shooterStopSwitch = new DigitalInput(2);
	public boolean currentLoaderMode = false;
	public long shootButtonTime = 0;
}
