package org.usfirst.frc.team2339.Barracuda.swervemath;

import edu.wpi.first.wpilibj.RobotDrive.MotorType;

public class WorkingPalmdale {
	public static final int kMaxNumberOfMotors = 4;
	
	// These should be set to actual robot dimensions. 
	// The units do not matter as long as they are consistent. Numbers below are in inches.
	public static final double WHEEL_BASE_LENGTH = 28.5;
	public static final double WHEEL_BASE_WIDTH = 25.0;
	// Distance of center of container in front of front wheels. (Negative if behind front wheel line.)
	public static final double CONTAINER_CENTER_DISTANCE_FORWARD = 2.0;

	public static final int frontLeft = MotorType.kFrontLeft.value;
    public static final int frontRight = MotorType.kFrontRight.value;
    public static final int rearLeft = MotorType.kRearLeft.value;
    public static final int rearRight = MotorType.kRearRight.value;
    
    public static class WheelVelocityVector {
    	public double wheelSpeed = 0;
    	public double wheelAngle = 0;
    	
    	public WheelVelocityVector() {
        	this.wheelSpeed = 0;
        	this.wheelAngle = 0;
    	}

    	public WheelVelocityVector(double wheelSpeed, double wheelAngle) {
        	this.wheelSpeed = wheelSpeed;
        	this.wheelAngle = wheelAngle;
    	}
    }
    
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
        	wheelAngles[frontLeft] = -45;
        	wheelAngles[rearRight] = -45;
    	}
    }

    /**
     * Calculate wheel velocity vector given wheel position relative to pivot and desired forward, strafe, and rotational velocities.
     * Wheel speed are normalized to the range [0, 1.0]. Angles are normalized to the range [-180, 180).
     * @see https://docs.google.com/presentation/d/1J_BajlhCQ236HaSxthEFL2PxywlneCuLNn276MWmdiY/edit?usp=sharing
     * @param xWheelPosition distance of wheel to right of pivot (left is negative).
     * @param yWheelPosition distance of wheel in front of pivot (back is negative).
     * @param maxWheelRadius distance of furtherest wheel from pivot.
     * @param xVelocity strafe (sideways) velocity. -1.0 = max motor speed left. 1.0 = max motor speed right.
     * @param yVelocity forward velocity. -1.0 = max motor speed backwards. 1.0 = max motor speed forward.
     * @param rotateVelocity clockwise rotational velocity. -1.0 = max motor speed counter-clockwise. 1.0 = max motor speed clockwise.
     * @return wheel vector (speed and angle)
     */
    public WheelVelocityVector calculateWheelVelocityVector(double xWheelPosition, double yWheelPosition, double maxWheelRadius, 
    		double xVelocity, double yVelocity, double rotateVelocity) {
        double xWheel = xVelocity - rotateVelocity * yWheelPosition / maxWheelRadius; 
        double yWheel = yVelocity - rotateVelocity * xWheelPosition / maxWheelRadius;
        WheelVelocityVector wheelVelocity = new WheelVelocityVector();
        wheelVelocity.wheelSpeed = Math.hypot(xWheel, yWheel);
        wheelVelocity.wheelAngle = Math.toDegrees(Math.atan2(xWheel, yWheel));
        return wheelVelocity;
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
    	
        double L = WHEEL_BASE_LENGTH;
        double W = WHEEL_BASE_WIDTH;
        double frontDist = L/2 - xPivotOffset; 
        double rearDist = L/2 + xPivotOffset; 
        double rightDist = W/2 - yPivotOffset;
        double leftDist = W/2 + yPivotOffset;
        
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
     * NOTE: This should give same result as standard method below.
     * Calculate raw wheel speeds and angles for swerve drive based on input robot forward, strafe, and rotational velocities.
     * Wheel speeds are normalized to the range [0, 1.0]. Angles are normalized to the range [-180, 180).
     * Calculated values are raw in that they have no consideration for current state of drive.
     * @param xVelocity strafe (sideways) velocity. -1.0 = max motor speed left. 1.0 = max motor speed right.
     * @param yVelocity forward velocity. -1.0 = max motor speed backwards. 1.0 = max motor speed forward.
     * @param rotateVelocity clockwise rotational velocity. -1.0 = max motor speed counter-clockwise. 1.0 = max motor speed clockwise.
     * @return raw wheel speeds and angles
     */
    public WheelData calculateRawWheelData1(double xVelocity, double yVelocity, double rotateVelocity) {
    	return calculateRawWheelDataGeneral(xVelocity, yVelocity, rotateVelocity, 0.0, 0.0);
    }
    
    public static void normalize(double wheelSpees[]) {
    	
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
        double L = WHEEL_BASE_LENGTH;
        double W = WHEEL_BASE_WIDTH;;
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
    public WheelData calculateDeltaWheelData(WheelData currentWheelData, WheelData rawWheelData) {
    	WheelData deltaWheelData = new WheelData();
    	for (int iiWheel = 0; iiWheel < kMaxNumberOfMotors; iiWheel++) {
    		// Compute turn angle from encoder value (pidGet) and raw target value
    		System.out.println("Wheel " + iiWheel + " current angle " + currentWheelData.wheelAngles[iiWheel]);
    		AngleFlip turnAngle = computeTurnAngle(currentWheelData.wheelAngles[iiWheel], rawWheelData.wheelAngles[iiWheel]);
            deltaWheelData.wheelAngles[iiWheel] = turnAngle.getAngle();
            deltaWheelData.wheelSpeeds[iiWheel] = driveScale(turnAngle) * rawWheelData.wheelSpeeds[iiWheel];
    	}
    	return deltaWheelData;
    }
    
    /**
     * Class to store angle and flip together
     * @author emiller
     *
     */
    public static class AngleFlip {
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
    public static double normalizeAngle(double theta) {
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
    public static AngleFlip computeTurnAngle(double currentAngle, double targetAngle) {
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
    public static double computeChangeAngle(double currentAngle, double targetAngle) {
    	return computeTurnAngle(currentAngle, targetAngle).getAngle();
    }
    
    /**
     * Scale drive speed based on how far wheel needs to turn
     * @param turnAngle Angle wheel needs to turn (with flip value)
     * @return speed scale factor in range [0, 1]
     */
    public static double driveScale(AngleFlip turnAngle) {
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
    

}
