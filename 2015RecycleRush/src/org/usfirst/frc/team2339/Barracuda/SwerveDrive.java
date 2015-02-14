package org.usfirst.frc.team2339.Barracuda;

/*
 * Add a swerve mode to RobotDrive
 * Code from Chief Delphi: http://www.chiefdelphi.com/forums/showthread.php?t=117099
 */


import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap;

//import com.sun.squawk.util.MathUtils;
import java.lang.Math;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 *
 * 
 */
public class SwerveDrive extends RobotDrive {
    //declare the steering pods and shifter valve

    public static final int frontLeft = MotorType.kFrontLeft.value;
    public static final int frontRight = MotorType.kFrontRight.value;
    public static final int rearLeft = MotorType.kRearLeft.value;
    public static final int rearRight = MotorType.kRearRight.value;
    
    protected SpeedController speedControllers[] = new SpeedController[kMaxNumberOfMotors];
    protected Pod wheelPods[] = new Pod[kMaxNumberOfMotors];
    private DoubleSolenoid shift;

    public class WheelData {
    	public double wheelSpeeds[] = new double[kMaxNumberOfMotors];
    	public double wheelAngles[] = new double[kMaxNumberOfMotors];
    	
    	public WheelData() {
    		// Initialize data
        	for (int iiWheel = 0; iiWheel < kMaxNumberOfMotors; iiWheel++) {
                wheelSpeeds[iiWheel] = 0;
                wheelAngles[iiWheel] = 0;
        	}
		}
    	
    	/**
    	 * Set speed and angle values when joystick in dead band
    	 */
    	public void setDeadBandValues() {
        	for (int iiWheel = 0; iiWheel < kMaxNumberOfMotors; iiWheel++) {
        		wheelSpeeds[iiWheel] = 0;
        		wheelAngles[iiWheel] = 45;
        	}
        	wheelAngles[frontRight] = -45;
        	wheelAngles[rearLeft] = -45;
    	}
    }
    
    public SwerveDrive() {
    	super(SwerveMap.PWM.DRIVE_FRONT_LEFT, 
    		  SwerveMap.PWM.DRIVE_REAR_LEFT, 
    		  SwerveMap.PWM.DRIVE_FRONT_RIGHT, 
    		  SwerveMap.PWM.DRIVE_REAR_RIGHT);
    	speedControllers[frontLeft] = new Talon(SwerveMap.PWM.DRIVE_FRONT_LEFT_STEERING);
    	speedControllers[frontRight] = new Talon(SwerveMap.PWM.DRIVE_FRONT_RIGHT_STEERING);
    	speedControllers[rearLeft] = new Talon(SwerveMap.PWM.DRIVE_REAR_LEFT_STEERING);
    	speedControllers[rearRight] = new Talon(SwerveMap.PWM.DRIVE_REAR_RIGHT_STEERING);
        //set up the steering pods with the correct sensors and controllers
        shift = new DoubleSolenoid(SwerveMap.Solenoid.DRIVE_SHIFT_HIGH, SwerveMap.Solenoid.DRIVE_SHIFT_LOW);
        wheelPods[frontLeft] = new Pod(m_frontLeftMotor,
        		speedControllers[frontLeft],
                SwerveMap.DIO.DRIVE_FRONT_LEFT_ENC_A,
                SwerveMap.DIO.DRIVE_FRONT_LEFT_ENC_B, 1);
        wheelPods[frontRight] = new Pod(m_frontRightMotor,
        		speedControllers[frontRight],
                SwerveMap.DIO.DRIVE_FRONT_RIGHT_ENC_A,
                SwerveMap.DIO.DRIVE_FRONT_RIGHT_ENC_B, 2);
        wheelPods[rearLeft] = new Pod(m_rearLeftMotor,
        		speedControllers[rearLeft],
                SwerveMap.DIO.DRIVE_REAR_LEFT_ENC_A,
                SwerveMap.DIO.DRIVE_REAR_LEFT_ENC_B, 3);
        wheelPods[rearRight] = new Pod(m_rearRightMotor,
        		speedControllers[rearRight],
                SwerveMap.DIO.DRIVE_REAR_RIGHT_ENC_A,
                SwerveMap.DIO.DRIVE_REAR_RIGHT_ENC_B, 4);
    }
    
    /**
     * Calculate raw speeds and angles for swerve drive.
     * Wheel speeds are normalized to the range [-1.0, 1.0]. Angles are normalized to the range [-180, 180).
     * Calculated values are raw in that they have no consideration for current state of drive.
     * @param x forward speed between -1.0 and 1.0
     * @param y side speed between -1.0 and 1.0
     * @param rotate rotation speed between -1.0 and 1.0
     * @return raw wheel speeds and angles
     */
    public WheelData calculateRawWheelData(double x, double y, double rotate) {
    	
    	WheelData rawWheelData = new WheelData();
    	
        //calculate angle/speed setpoints using wheel dimensions from SwerveMap 
        double L = SwerveMap.Constants.WHEEL_BASE_LENGTH;
        double W = SwerveMap.Constants.WHEEL_BASE_WIDTH;;
        double R = Math.sqrt((L * L) + (W * W));
        double A = x - rotate * (L / R);
        double B = x + rotate * (L / R);
        double C = y - rotate * (W / R);
        double D = y + rotate * (W / R);
        
        // Find wheel speeds
        rawWheelData.wheelSpeeds[frontLeft] = Math.sqrt((B * B) + (D * D));
        rawWheelData.wheelSpeeds[frontRight] = Math.sqrt((B * B) + (C * C));
        rawWheelData.wheelSpeeds[rearLeft] = Math.sqrt((A * A) + (D * D));
        rawWheelData.wheelSpeeds[rearRight] = Math.sqrt((A * A) + (C * C));
        
        normalize(rawWheelData.wheelSpeeds);
        
        // Find steering angles
        rawWheelData.wheelAngles[frontLeft] = Math.toDegrees(Math.atan2(B, D));
        rawWheelData.wheelAngles[frontRight] = Math.toDegrees(Math.atan2(B, C));
        rawWheelData.wheelAngles[rearLeft] = Math.toDegrees(Math.atan2(A, D));
        rawWheelData.wheelAngles[rearRight] = Math.toDegrees(Math.atan2(A, C));
        
        return rawWheelData;
    }
    
    /**
     * Calculate wheel data change (delta) based on current data.
     * @param rawWheelData Raw wheel change data
     * @return wheel change data (delta) based on current wheel values
     */
    public WheelData calculateDeltaWheelData(WheelData rawWheelData) {
    	WheelData deltaWheelData = new WheelData();
    	for (int iiWheel = 0; iiWheel < kMaxNumberOfMotors; iiWheel++) {
    		// Compute turn angle from encoder value (pidGet) and raw target value
    		AngleFlip turnAngle = computeTurnAngle(wheelPods[iiWheel].pidGet(), rawWheelData.wheelAngles[iiWheel]);
            deltaWheelData.wheelAngles[iiWheel] = turnAngle.getAngle();
            deltaWheelData.wheelSpeeds[iiWheel] = driveScale(turnAngle) * rawWheelData.wheelSpeeds[iiWheel];
    	}
    	return deltaWheelData;
    }
    
    
    public void setPods(WheelData wheelData) {
    	for (int iiWheel = 0; iiWheel < kMaxNumberOfMotors; iiWheel++) {
            wheelPods[iiWheel].setSteeringAngle(wheelData.wheelAngles[iiWheel]);
            wheelPods[iiWheel].setWheelSpeed(wheelData.wheelSpeeds[iiWheel]);
    	}
    }

    /**
     * Drive in swerve mode with a given speed, rotation, and shift values.
     * Driving parameters are assumed to be relative to the current robot angle.
     * @param x forward speed between -1.0 and 1.0
     * @param y side speed between -1.0 and 1.0
     * @param rotate rotation speed between -1.0 and 1.0
     * @param isLowGear true if need to shift to low
     * @param isHighGear true if need to shift to high
     */
    public void swerveDriveRobot(double x, double y, double rotate, 
    		boolean isLowGear, boolean isHighGear) {
    	
    	
    	WheelData deltaWheelData = null;
    	if (x > SwerveMap.Control.DRIVE_STICK_DEAD_BAND || y > SwerveMap.Control.DRIVE_STICK_DEAD_BAND || 
    			rotate > SwerveMap.Control.DRIVE_STICK_DEAD_BAND) {
    		// Compute new values
        	WheelData rawWheelData = calculateRawWheelData(x, y, rotate);
    		deltaWheelData = calculateDeltaWheelData(rawWheelData);
    	} else {
    		// Joystick in dead band, set neutral values
    		deltaWheelData = new WheelData();
    		deltaWheelData.setDeadBandValues();
    	}
    	
        // Set shifter
        if(isLowGear){
            shift.set(DoubleSolenoid.Value.kForward);
        }
        if(isHighGear){
            shift.set(DoubleSolenoid.Value.kReverse);
        }
        
        // Set pods
        setPods(deltaWheelData);

    }

    /**
     * Drive in swerve mode with a given speed, rotation, and shift values.
     * Driving parameters are assumed to be absolute based on a fixed angle, e.g. the field.
     * @param robotAngle Angle (in degrees) of robot relative to fixed angle. This is probably taken from the gyro.
     * @param x forward speed between -1.0 and 1.0
     * @param y side speed between -1.0 and 1.0
     * @param rotate rotation speed between -1.0 and 1.0
     * @param isLowGear true if need to shift to low
     * @param isHighGear true if need to shift to high
     */
    public void swerveDriveAbsolute(double robotAngle, double x, double y, double rotate,  
    		boolean isLowGear, boolean isHighGear) {
    	double robotAngleRad = Math.toRadians(robotAngle);
    	double xRobot = -x * Math.sin(robotAngleRad) + y * Math.cos(robotAngleRad);
    	double yRobot = x * Math.cos(robotAngleRad) + y * Math.sin(robotAngleRad);
    	this.swerveDriveRobot(xRobot, yRobot, rotate, isLowGear, isHighGear);
    }
    
    /**
     * Control robot relative to itself
     */
    public void swerveDriveTeleop() {
        double x, y, rotate;
        boolean isLowGear, isHighGear;
        x = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK);
        y = -SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_SIDEWAYS);
        rotate = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE);
        isLowGear = SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_CONTROLLER_SHIFT_LOW);
        isHighGear = SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_CONTROLLER_SHIFT_HIGH);
        swerveDriveRobot(x, y, rotate, isLowGear, isHighGear);
    }
    
    /**
     * Control robot relative to a fixed angle using gyro
     */
    public void swerveDriveTeleopGyro() {
        double x, y, rotate;
        boolean isLowGear, isHighGear;
        x = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK);
        y = -SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_SIDEWAYS);
        rotate = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE);
        isLowGear = SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_CONTROLLER_SHIFT_LOW);
        isHighGear = SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_CONTROLLER_SHIFT_HIGH);
        double robotAngle = SwerveMap.Control.GYRO.getAngle();
        swerveDriveAbsolute(robotAngle, x, y, rotate, isLowGear, isHighGear);
    }
    
    /**
     * Class to store angle and flip together
     * @author emiller
     *
     */
    public class AngleFlip {
    	private double angle;
    	private boolean flip;
    	
    	public AngleFlip() {
    		setAngle(0);
    		setFlip(false);
    	}
    	public AngleFlip(double angle) {
    		this.setAngle(angle);
    		setFlip(false);
    	}
    	public AngleFlip(double angle, boolean flip) {
    		this.setAngle(angle);
    		flip = false;
    	}
		/**
		 * @return the angle
		 */
		public double getAngle() {
			return angle;
		}
		/**
		 * @param angle the angle to set
		 */
		public void setAngle(double angle) {
			this.angle = angle;
		}
		/**
		 * @return the flip
		 */
		public boolean isFlip() {
			return flip;
		}
		/**
		 * @param flip the flip to set
		 */
		public void setFlip(boolean flip) {
			this.flip = flip;
		}
    };
    
    /** 
     * Normalizes an angle in degrees to (-180, 180].
     * @param theta Angle to normalize
     * @return Normalized angle
     */
    public double normalizeAngle(double theta) {
    	while (theta > 180) {
    		theta -= 360;
    	}
    	while (theta < -180) {
    		theta += 360;
    	}
    	return theta;
    }

    /**
     * Compute angle needed to turn and whether or not flip is needed
     * @param currentAngle
     * @param targetAngle
     * @return new angle with flip
     */
    public AngleFlip computeTurnAngle(double currentAngle, double targetAngle) {
    	AngleFlip turnAngle = new AngleFlip(targetAngle - currentAngle, false);
    	if (Math.abs(turnAngle.getAngle()) > 90) {
    		turnAngle.setAngle(normalizeAngle(turnAngle.getAngle() + 180));
    		turnAngle.setFlip(true);
    	}
    	return turnAngle;
    }
    
    /**
     * Compute change angle to get from current to target angle.
     * @param currentAngle Current angle
     * @param targetAngle New angle to change to
     * @return change angle
     */
    public double computeChangeAngle(double currentAngle, double targetAngle) {
    	return computeTurnAngle(currentAngle, targetAngle).getAngle();
    }
    
    /**
     * Scale drive speed based on how far wheel needs to turn
     * @param turnAngle Angle wheel needs to turn (with flip value)
     * @return speed scale factor in range [0, 1]
     */
    public double driveScale(AngleFlip turnAngle) {
    	double scale = 0;
    	if (Math.abs(turnAngle.getAngle()) < 45) {
    		/*
    		 * Eric comment: I don't like the discontinuous nature of this scaling.
    		 * Possible improvements:
    		 *   1) Use cosine(2 * turnAngle)
    		 *   2) Scale any angle < 90.
    		 */
    		scale = Math.cos(Math.toRadians(turnAngle.getAngle()));
    	} else {
    		scale = 0;
    	}
    	if (turnAngle.isFlip()) {
    		scale = -scale;
    	}
    	return scale;
    }
    
    private class Pod implements PIDOutput, PIDSource {

        private Encoder steeringEnc;
        private SpeedController drive;
        private SpeedController steer;
        private PIDController pid;

        public Pod(SpeedController driveController, SpeedController steeringController, int steeringEncA,
                int steeringEncB, int podNumber) {
            steeringEnc = new Encoder(steeringEncA, steeringEncB);
            steeringEnc.setDistancePerPulse(SwerveMap.Constants.STEERING_ENC_PULSES_PER_REVOLUTION);
            drive = driveController;
            steer = steeringController;
            pid = new PIDController(SwerveMap.Constants.STEERING_PID_P,
                    SwerveMap.Constants.STEERING_PID_I,
                    SwerveMap.Constants.STEERING_PID_D, this, this);
            SmartDashboard.putData("Steering Pod " + podNumber, pid);
            pid.setInputRange(-180, 180);
            pid.setContinuous(true);
            pid.enable();

        }

        public void pidWrite(double output) {
            steer.set(output);
        }

        public double pidGet() {
            return steeringEnc.getDistance();
        }

        public void setSteeringAngle(double angle) {
            pid.setSetpoint(angle);
        }

        public void setWheelSpeed(double speed) {
            drive.set(speed);
        }
    }

    public void initDefaultCommand() {
    }
}