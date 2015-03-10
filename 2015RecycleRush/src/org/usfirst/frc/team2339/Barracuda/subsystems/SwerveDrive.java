package org.usfirst.frc.team2339.Barracuda.subsystems;

/*
 * Add a swerve mode to RobotDrive
 * Code from Chief Delphi: http://www.chiefdelphi.com/forums/showthread.php?t=117099
 */


import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap;


import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap.Constants;
import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap.Control;
import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap.DIO;
import org.usfirst.frc.team2339.Barracuda.RobotMap.SwerveMap.PWM;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.RectangularCoordinates;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.RobotMotion;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.VelocityPolar;



//import com.sun.squawk.util.MathUtils;
import java.lang.Math;

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
public class SwerveDrive {

	/*
	 * Wheel are numbered in counter-clockwise order when viewed from top of robot.
	 * For a typical four wheel configuration the front right wheel is Number 0.
	 * This follows the scheme in Ether's derivation of swerve inverse kinematics.
	 * @See http://www.chiefdelphi.com/media/papers/2426
	 * 
	 * The static values provided below are for convenience.
	 * Note: Ether starts numbering at 1, but Java indices start at zero. 
	 *       Thus the indices below start at zero.
	 * 
	 */
    public static final int frontRight = 0;
    public static final int frontLeft = 1;
    public static final int rearLeft = 2;
    public static final int rearRight = 3;
    
    protected final SwerveWheelDrive wheels[];
    protected RectangularCoordinates pivot = new RectangularCoordinates(0, 0);

	// Distance of wheel furthest from pivot
    protected double maxWheelRadius = 1;
    
    public SwerveDrive(SwerveWheelDrive wheels[]) {
    	this.wheels = new SwerveWheelDrive[wheels.length];
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		this.wheels[iiWheel] = wheels[iiWheel];
    	}
    	setPivot(new RectangularCoordinates(0, 0));
    }
    
    public double getMaxWheelRadius() {
		return maxWheelRadius;
	}

	public void setMaxWheelRadius() {
		maxWheelRadius = 0;
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		double radius = wheels[iiWheel].getWheelPosition().subtract(pivot).magnitude();
    		if (radius > maxWheelRadius) {
    			maxWheelRadius = radius;
    		}
    	}
	}

    public RectangularCoordinates getPivot() {
		return pivot;
	}

	public void setPivot(RectangularCoordinates pivot) {
		this.pivot = pivot;
		setMaxWheelRadius();
	}
	
    /**
     * Calculate raw wheel speeds and angles for swerve drive based on input robot forward, strafe, and rotational velocities.
     * Wheel speeds are normalized to the range [0, 1.0]. Angles are normalized to the range [-180, 180).
     * Calculated values are raw in that they have no consideration for current state of drive.
     * Most swerve code assumes the pivot point for rotation is the center of the wheels (i.e. center of rectangle with wheels as corners)
     * This calculation is generalized based on pivot being offset from rectangle center.
     * @param xVelocity strafe (sideways) velocity. -1.0 = max motor speed left. 1.0 = max motor speed right.
     * @param yVelocity forward velocity. -1.0 = max motor speed backwards. 1.0 = max motor speed forward.
     * @param rotateVelocity clockwise rotational velocity. -1.0 = max motor speed counter-clockwise. 1.0 = max motor speed clockwise.
     * @param xPivotOffset Amount pivot is offset sideways from center. (Positive toward right, negative toward left)
     * @param yPivotOffset Amount pivot is offset forward from center. (Positive toward front, negative toward back)
     * @return raw wheel speeds and angles
     */
    public WheelData calculateRawWheelDataGeneral(double xVelocity, double yVelocity, double rotateVelocity, 
    		double xPivotOffset, double yPivotOffset) {
    	
    	WheelData rawWheelData = new WheelData();
    	
        double L = SwerveMap.Constants.WHEEL_BASE_LENGTH;
        double W = SwerveMap.Constants.WHEEL_BASE_WIDTH;
        double frontDist = L/2 - yPivotOffset; 
        double rearDist = L/2 + yPivotOffset; 
        double rightDist = W/2 - xPivotOffset;
        double leftDist = W/2 + xPivotOffset;
        
        // Find maximum wheel distance (radius) from center
        // Maximum radius is used to normalize rotational velocity so that wheels farthest from center move the fastest.
        double xMax = Math.max(rightDist, leftDist);
        double yMax = Math.max(frontDist, rearDist);
        double rMax = Math.hypot(xMax, yMax);

        WheelVelocityVector wheelVelocity = new WheelVelocityVector();
        
        wheelVelocity = this.calculateWheelVelocityVector(rightDist, frontDist, rMax, xVelocity, yVelocity, rotateVelocity);
        rawWheelData.wheelSpeeds[frontRight] = wheelVelocity.wheelSpeed;
        rawWheelData.wheelAngles[frontRight] = wheelVelocity.wheelAngle;
        
        wheelVelocity = this.calculateWheelVelocityVector(-leftDist, frontDist, rMax, xVelocity, yVelocity, rotateVelocity);
        rawWheelData.wheelSpeeds[frontLeft] = wheelVelocity.wheelSpeed;
        rawWheelData.wheelAngles[frontLeft] = wheelVelocity.wheelAngle;
        
        wheelVelocity = this.calculateWheelVelocityVector(-leftDist, -rearDist, rMax, xVelocity, yVelocity, rotateVelocity);
        rawWheelData.wheelSpeeds[rearLeft] = wheelVelocity.wheelSpeed;
        rawWheelData.wheelAngles[rearLeft] = wheelVelocity.wheelAngle;
        
        wheelVelocity = this.calculateWheelVelocityVector(rightDist, -rearDist, rMax, xVelocity, yVelocity, rotateVelocity);
        rawWheelData.wheelSpeeds[rearRight] = wheelVelocity.wheelSpeed;
        rawWheelData.wheelAngles[rearRight] = wheelVelocity.wheelAngle;

        // Normalize all wheel speeds to be <= 1.0
        normalize(rawWheelData.wheelSpeeds);
        
        return rawWheelData;
    }
    
    /**
     * Calculate raw wheel speeds and angles for swerve drive based on input robot forward, strafe, and rotational velocities.
     * Wheel speeds are normalized to the range [0, 1.0]. Angles are normalized to the range [-180, 180).
     * Calculated values are raw in that they have no consideration for current state of drive.
     * @param xVelocity strafe (sideways) velocity. -1.0 = max motor speed left. 1.0 = max motor speed right.
     * @param yVelocity forward velocity. -1.0 = max motor speed backwards. 1.0 = max motor speed forward.
     * @param rotateVelocity clockwise rotational velocity. -1.0 = max motor speed counter-clockwise. 1.0 = max motor speed clockwise.
     * @return raw wheel speeds and angles
     */
    public WheelData calculateRawWheelData(double xVelocity, double yVelocity, double rotateVelocity) {
    	
    	WheelData rawWheelData = new WheelData();
    	
        //calculate angle/speed setpoints using wheel dimensions from SwerveMap 
        double L = SwerveMap.Constants.WHEEL_BASE_LENGTH;
        double W = SwerveMap.Constants.WHEEL_BASE_WIDTH;;
        double R = Math.hypot(L, W);
        double A = xVelocity - rotateVelocity * (L / R);
        double B = xVelocity + rotateVelocity * (L / R);
        double C = yVelocity - rotateVelocity * (W / R);
        double D = yVelocity + rotateVelocity * (W / R);
        
        // Find wheel speeds
        rawWheelData.wheelSpeeds[frontLeft] = Math.hypot(B, D);
        rawWheelData.wheelSpeeds[frontRight] = Math.hypot(B, C);
        rawWheelData.wheelSpeeds[rearLeft] = Math.hypot(A, D);
        rawWheelData.wheelSpeeds[rearRight] = Math.hypot(A, C);
        
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
    		SmartDashboard.putNumber("Wheel " + iiWheel + " current angle ", wheelPods[iiWheel].pidGet());
    		AngleFlip turnAngle = computeTurnAngle(wheelPods[iiWheel].pidGet(), rawWheelData.wheelAngles[iiWheel]);
    		double targetAngle = normalizeAngle(wheelPods[iiWheel].pidGet() + turnAngle.getAngle()); 
            deltaWheelData.wheelAngles[iiWheel] = targetAngle;
            deltaWheelData.wheelSpeeds[iiWheel] = driveScale(turnAngle) * rawWheelData.wheelSpeeds[iiWheel];
    	}
    	return deltaWheelData;
    }
    
    /**
     * Drive in swerve mode with a given speed and rotation.
     * Driving parameters are assumed to be relative to the current robot angle.
     * @param robotMotion desired motion of robot express by strafe, frontBack, and rotation around a pivot point.
     * @param pivot Position of pivot. 
     */
    public void swerveDriveRobot(
    		RobotMotion robotMotion, 
    		RectangularCoordinates pivot) {
    	
    	setPivot(pivot);
    	
    	VelocityPolar rawVelocities[] = new VelocityPolar[wheels.length];
    	double speedMax = 0;
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		rawVelocities[iiWheel] = wheels[iiWheel].calculateWheelVelocity(getPivot(), maxWheelRadius, robotMotion);
    		if (Math.abs(rawVelocities[iiWheel].speed) > speedMax) {
    			speedMax = rawVelocities[iiWheel].speed;
    		}
    	}
    	
    	if (speedMax > 1.0) {
	    	// Normalize speeds to less than |1.0|
	    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
	    		rawVelocities[iiWheel].speed /= speedMax;
	    	}
    	}
    	
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		wheels[iiWheel].setWheelSanely(rawVelocities[iiWheel]);
    	}
    }

    /**
     * Drive in swerve mode with a given speed and rotation.
     * Driving parameters are assumed to be absolute based on a fixed angle, e.g. the field.
     * @param xVelocity strafe (sideways) velocity. -1.0 = max motor speed left. 1.0 = max motor speed right.
     * @param robotAngle Angle (in degrees) of robot relative to fixed angle. Zero degrees means front of robot points in desired direction. 
     *                   Positive is clockwise, negative counter-clockwise. This is probably taken from the gyro.
     * @param yVelocity forward velocity. -1.0 = max motor speed backwards. 1.0 = max motor speed forward.
     * @param rotateVelocity clockwise rotational velocity. -1.0 = max motor speed counter-clockwise. 1.0 = max motor speed clockwise.
     * @param xPivotOffset Amount pivot is offset sideways from center. (Positive toward right, negative toward left)
     * @param yPivotOffset Amount pivot is offset forward from center. (Positive toward front, negative toward back)
     */
    public void swerveDriveAbsolute(
    		RobotMotion robotMotion, 
    		double robotAngle,
    		RectangularCoordinates pivot) {
    	double robotAngleRad = Math.toRadians(robotAngle);
    	RobotMotion relativeMotion = new RobotMotion(
    			robotMotion.strafe * Math.cos(robotAngleRad) - robotMotion.frontBack * Math.sin(robotAngleRad),
    			robotMotion.strafe * Math.sin(robotAngleRad) + robotMotion.frontBack * Math.cos(robotAngleRad), 
    			robotMotion.rotate);
    	this.swerveDriveRobot(relativeMotion, pivot);
    }
    
	/**
	 * Set speed and angle values when joystick in dead band
	 */
	public void setDeadBandValues() {
    	for (int iiWheel = 0; iiWheel < wheels.length; iiWheel++) {
    		if (iiWheel == frontLeft || iiWheel == rearRight) {
        		wheels[iiWheel].setWheel(new VelocityPolar(0, -45));
    		} else {
        		wheels[iiWheel].setWheel(new VelocityPolar(0, 45));
    		}
    	}
	}
	
    /**
     * Control robot using joystick inputs
     */
    public void swerveDriveTeleop() {
    	
        double xVelocity, yVelocity, rotateVelocity;
        yVelocity = -SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_FORWARD_BACK);
        xVelocity = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_SIDEWAYS);
        rotateVelocity = SwerveMap.Control.DRIVE_STICK.getRawAxis(SwerveMap.Control.DRIVE_AXIS_ROTATE);
        
    	if (Math.abs(xVelocity) > SwerveMap.Control.DRIVE_STICK_DEAD_BAND || Math.abs(yVelocity) > SwerveMap.Control.DRIVE_STICK_DEAD_BAND || 
    			Math.abs(rotateVelocity) > SwerveMap.Control.DRIVE_STICK_DEAD_BAND) {
            rotateVelocity = (.5 * rotateVelocity);
            double xPivotOffset = 0.0;
            double yPivotOffset = 0.0;
            if (SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_BUTTON_ROTATE_AROUND_CONTAINER)) {
            	xPivotOffset = 0.0;
            	yPivotOffset = SwerveMap.Constants.CONTAINER_CENTER_DISTANCE_FORWARD + 0.5 * SwerveMap.Constants.WHEEL_BASE_LENGTH;
            }
            if (SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_BUTTON_SPEED_SHIFT)) {
              	 rotateVelocity = ( .5 * rotateVelocity );
              	 xVelocity = ( .5 * xVelocity);
              	 yVelocity = ( .5 * yVelocity);
               }
            double robotAngle = 0.0;
            if (SwerveMap.Control.DRIVE_STICK.getRawButton(SwerveMap.Control.DRIVE_BUTTON_ABSOLUTE_GYRO_MODE)) {
                robotAngle = SwerveMap.Control.GYRO.getAngle();
            }
            
            swerveDriveAbsolute(xVelocity, yVelocity, robotAngle, rotateVelocity, xPivotOffset, yPivotOffset);
    	} else {
    		// Joystick in dead band, set neutral values
    		WheelData wheelData = new WheelData();
    		setDeadBandValues();
    	}
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
    
    public class Pod implements PIDOutput, PIDSource {

        private Encoder steeringEnc;
        private SpeedController drive;
        private SpeedController steer;
        private PIDController pid;
        private int podNumber;

        public Pod(SpeedController driveController, SpeedController steeringController, int steeringEncA,
                int steeringEncB, int podNumber) {
            steeringEnc = new Encoder(steeringEncA, steeringEncB);
            steeringEnc.setDistancePerPulse(SwerveMap.Constants.STEERING_ENC_DEGREES_PER_PULSE);
            drive = driveController;
            steer = steeringController;
            this.podNumber = podNumber;
            pid = new PIDController(SwerveMap.Constants.STEERING_PID_P,
                    SwerveMap.Constants.STEERING_PID_I,
                    SwerveMap.Constants.STEERING_PID_D, this, this);
            SmartDashboard.putData("Steering Pod " + podNumber, pid);
            pid.setInputRange(-180, 180);
            pid.setContinuous(true);
        }

        public void pidWrite(double output) {
            steer.set(output);
        }

        public double pidGet() {
        	SmartDashboard.putData("Enc " + this.podNumber + " ", steeringEnc);
            return steeringEnc.getDistance();
        }

        public void setSteeringAngle(double angle) {
            pid.setSetpoint(angle);
        }
        public void setWheelSpeed(double speed) {
            drive.set(speed);
       // public void setWheelSpeed(double speed) {
         //   drive.set(.5 * speed);
        }
        
        public void resetAngle() {
        	steeringEnc.reset();
        }
        
        public void pidEnable(boolean enable) {
        	if (enable) {
        		pid.enable();
        	} else {
        		pid.disable();
        	}
        }
    }

    public void initDefaultCommand() {
    }

}