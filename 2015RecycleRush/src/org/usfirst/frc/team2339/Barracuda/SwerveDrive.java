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

    protected SpeedController m_frontLeftSteeringMotor;
    protected SpeedController m_frontRightSteeringMotor;
    protected SpeedController m_rearLeftSteeringMotor;
    protected SpeedController m_rearRightSteeringMotor;
    private Pod frontLeft, frontRight, rearLeft, rearRight;
    private DoubleSolenoid shift;

    public SwerveDrive() {
    	super(SwerveMap.PWM.DRIVE_FRONT_LEFT, 
    		  SwerveMap.PWM.DRIVE_REAR_LEFT, 
    		  SwerveMap.PWM.DRIVE_FRONT_RIGHT, 
    		  SwerveMap.PWM.DRIVE_REAR_RIGHT);
        m_frontLeftSteeringMotor = new Talon(SwerveMap.PWM.DRIVE_FRONT_LEFT_STEERING);
        m_rearLeftSteeringMotor = new Talon(SwerveMap.PWM.DRIVE_REAR_LEFT_STEERING);
        m_frontRightSteeringMotor = new Talon(SwerveMap.PWM.DRIVE_FRONT_RIGHT_STEERING);
        m_rearRightSteeringMotor = new Talon(SwerveMap.PWM.DRIVE_REAR_RIGHT_STEERING);
        //set up the steering pods with the correct sensors and controllers
        shift = new DoubleSolenoid(SwerveMap.Solenoid.DRIVE_SHIFT_HIGH, SwerveMap.Solenoid.DRIVE_SHIFT_LOW);
        frontLeft = new Pod(m_frontLeftMotor,
        		m_frontLeftSteeringMotor,
                SwerveMap.DIO.DRIVE_FRONT_LEFT_ENC_A,
                SwerveMap.DIO.DRIVE_FRONT_LEFT_ENC_B, 1);
        frontRight = new Pod(m_frontRightMotor,
        		m_frontRightSteeringMotor,
                SwerveMap.DIO.DRIVE_FRONT_RIGHT_ENC_A,
                SwerveMap.DIO.DRIVE_FRONT_RIGHT_ENC_B, 2);
        rearLeft = new Pod(m_rearLeftMotor,
        		m_rearLeftSteeringMotor,
                SwerveMap.DIO.DRIVE_REAR_LEFT_ENC_A,
                SwerveMap.DIO.DRIVE_REAR_LEFT_ENC_B, 3);
        rearRight = new Pod(m_rearRightMotor,
        		m_rearRightSteeringMotor,
                SwerveMap.DIO.DRIVE_REAR_RIGHT_ENC_A,
                SwerveMap.DIO.DRIVE_REAR_RIGHT_ENC_B, 4);
    }

    /**
     * Drive swerve with a given speed, rotation, and shift values
     * @param x forward speed between -1.0 and 1.0
     * @param y side speed between -1.0 and 1.0
     * @param rotate rotation between -1.0 and 1.0
     * @param isLowGear true if need to shift to low
     * @param isHighGear true if need to shift to high
     */
    public void swerveDrive(double x, double y, double rotate, boolean isLowGear, boolean isHighGear) {
        //calculate angle/speed setpoints using 28 by 38 inch robot 
        double L = SwerveMap.Constants.WHEEL_BASE_LENGTH;
        double W = SwerveMap.Constants.WHEEL_BASE_WIDTH;;
        double R = Math.sqrt((L * L) + (W * W));
        double A = x - rotate * (L / R);
        double B = x + rotate * (L / R);
        double C = y - rotate * (W / R);
        double D = y + rotate * (W / R);
        
        // Find wheel speeds
        double wheelSpeeds[] = new double[kMaxNumberOfMotors];
        wheelSpeeds[MotorType.kFrontLeft.value] = Math.sqrt((B * B) + (D * D));
        wheelSpeeds[MotorType.kFrontRight.value] = Math.sqrt((B * B) + (C * C));
        wheelSpeeds[MotorType.kRearLeft.value] = Math.sqrt((A * A) + (D * D));
        wheelSpeeds[MotorType.kRearRight.value] = Math.sqrt((A * A) + (C * C));
        
        normalize(wheelSpeeds);
        
        // Find steering angles
        double frontRightSteeringAngle = Math.atan2(B, C)*180/Math.PI;
        double frontLeftSteeringAngle = Math.atan2(B, D)*180/Math.PI;
        double rearLeftSteeringAngle = Math.atan2(A, D)*180/Math.PI;
        double rearRightSteeringAngle = Math.atan2(A, C)*180/Math.PI;
        
        // Set shifter
        if(isLowGear){
            shift.set(DoubleSolenoid.Value.kForward);
        }
        if(isHighGear){
            shift.set(DoubleSolenoid.Value.kReverse);
        }
        
        // Set pods
        frontRight.setSteeringAngle(frontRightSteeringAngle);
        frontRight.setWheelSpeed(wheelSpeeds[MotorType.kFrontRight.value]);
        frontLeft.setSteeringAngle(frontLeftSteeringAngle);
        frontLeft.setWheelSpeed(wheelSpeeds[MotorType.kFrontLeft.value]);
        rearLeft.setSteeringAngle(rearLeftSteeringAngle);
        rearLeft.setWheelSpeed(wheelSpeeds[MotorType.kRearLeft.value]);
        rearRight.setSteeringAngle(rearRightSteeringAngle);
        rearRight.setWheelSpeed(wheelSpeeds[MotorType.kRearRight.value]);

    }

    public void swerveDriveTeleop() {
        // Get values from joysticks
        double x, y, rotate;
        boolean isLowGear, isHighGear;
        x = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK);
        y = -SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_SIDEWAYS);
        rotate = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE);
        isLowGear = SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_CONTROLLER_SHIFT_LOW);
        isHighGear = SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_CONTROLLER_SHIFT_HIGH);
        swerveDrive(x, y, rotate, isLowGear, isHighGear);
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