package org.usfirst.frc.team2339.Barracuda.swervemath;



public class SwerveWheel {

	/**
     * Store rectangular (x and y) coordinates
     * Can represent a position or a vector
     * 
     * @author emiller
     *
     */
    public static class RectangularCoordinates {
    	public double x;
    	public double y;
    	
    	public RectangularCoordinates(double x, double y) {
    		this.x = x;
    		this.y = y;
    	}
    	
    	public RectangularCoordinates subtract(RectangularCoordinates p0) {
    		return new RectangularCoordinates(x - p0.x, y - p0.y);
    	}
    	
    	public double magnitude() {
    		return Math.sqrt(x * x + y * y);
    	}
    }
    
    /**
     * Store robot motion as strafe, front-back, and rotation.
     * strafe is sideways velocity with -1.0 = max motor speed left and 1.0 = max motor speed right. 
     * frontBack is forward velocity with -1.0 = max motor speed backwards and 1.0 = max motor speed forward.
     * rotate is clockwise rotational velocity. -1.0 = max motor speed counter-clockwise. 1.0 = max motor speed clockwise.
     * 
     * @author emiller
     *
     */
    public static class RobotMotion {
    	public double strafe;
    	public double frontBack;
    	public double rotate;
    	
    	public RobotMotion(double strafe, double frontBack, double rotate) {
    		this.strafe = strafe;
    		this.frontBack = frontBack;
    		this.rotate = rotate;
    	}
    }
    
    /**
     * Store a velocity as speed and angle
     * @author emiller
     *
     */
    public static class VelocityPolar {
    	public double speed = 0;
    	public double angle = 0;
    	
    	public VelocityPolar(double speed, double angle) {
    		this.speed = speed;
    		this.angle = angle;
    	}
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
    
	public double getRadialAngle(RectangularCoordinates wheelPosition) {
		return Math.toDegrees(Math.atan2(wheelPosition.x, wheelPosition.y));
	}
	
	public double getPerpendicularAngle(RectangularCoordinates wheelPosition) {
		return Math.toDegrees(Math.atan2(-wheelPosition.y, wheelPosition.x));
	}
	
    /**
     * Calculate wheel velocity vector given wheel position and pivot location. 
     * Robot motion is expressed with strafe, forward-back, and rotational velocities.
     * Wheel speed are normalized to the range [0, 1.0]. Angles are normalized to the range [-180, 180).
     * @see https://docs.google.com/presentation/d/1J_BajlhCQ236HaSxthEFL2PxywlneCuLNn276MWmdiY/edit?usp=sharing
     * 
     * @param wheelPosition Position of wheel. 
     *                      x is left-right, with right positive. y is front-back with front positive.
     * @param pivot Position of pivot. 
     * @param maxWheelRadius distance of furtherest wheel on robot from pivot.
     * @param robotMotion desired motion of robot express by strafe, frontBack, and rotation around a pivot point.
     * @return wheel polar velocity (speed and angle)
     */
    public static VelocityPolar calculateWheelVelocity(
    		RectangularCoordinates wheelPosition,
    		RectangularCoordinates pivot,
    		double maxWheelRadius, 
    		RobotMotion robotMotion) {
    	
    	RectangularCoordinates wheelRelativePosition = wheelPosition.subtract(pivot);
    	double rotateSpeed = robotMotion.rotate / maxWheelRadius;
    	RectangularCoordinates wheelVectorRobotCoord = new RectangularCoordinates(
    			robotMotion.strafe - rotateSpeed * wheelRelativePosition.y,  
    			robotMotion.frontBack + rotateSpeed * wheelRelativePosition.x);

    	double wheelSpeed = Math.hypot(wheelVectorRobotCoord.x, wheelVectorRobotCoord.y); 
    	// Clockwise
    	double wheelAngle = Math.toDegrees(Math.atan2(-wheelVectorRobotCoord.x, wheelVectorRobotCoord.y));
    	// Counter clockwise
    	// double wheelAngle = Math.toDegrees(Math.atan2(-wheelVectorRobotCoord.x, wheelVectorRobotCoord.y));
    	
        return new VelocityPolar(wheelSpeed, wheelAngle);
    }
    
    public static VelocityPolar[] calculateRectangularWheelVelocities(double length, double width, 
    		RobotMotion robotMotion, RectangularCoordinates pivot) {
    	
    	VelocityPolar rawVelocities[] = new VelocityPolar[4];
    	RectangularCoordinates wheelPosition = new RectangularCoordinates(0, 0);
		for (int iiWheel = 0; iiWheel < 4; iiWheel++) {
			switch(iiWheel) {
			case 0:
			default:
				wheelPosition.x = width/2;
				wheelPosition.y = length/2;
				break;
			case 1:
				wheelPosition.x = -width/2;
				wheelPosition.y = length/2;
				break;
			case 2:
				wheelPosition.x = -width/2;
				wheelPosition.y = -length/2;
				break;
			case 3:
				wheelPosition.x = width/2;
				wheelPosition.y = -length/2;
				break;
				
			}
    		rawVelocities[iiWheel] = calculateWheelVelocity(wheelPosition, pivot, 
    				Math.hypot(width/2, length/2), robotMotion);
    	}
		
		normalize(rawVelocities);
    	
    	return rawVelocities;
    }

    public static VelocityPolar[] calculateRectangularWheelVelocities(double length, double width, 
    		RobotMotion robotMotion) {
    	return calculateRectangularWheelVelocities(length, width, robotMotion, new RectangularCoordinates(0, 0));
    }
    	
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
    	if (Math.abs(turnAngle.getAngle()) <= 90) {
    		/*
    		 * Eric comment: I don't like the discontinuous nature of this scaling.
    		 * Possible improvements:
    		 *   1) Scale any angle < 90.
    		 */
    		scale = Math.cos(Math.toRadians(turnAngle.getAngle()));
    	} else {
    		scale = 0;
    	}
    	// Override above speed scaling.
    	scale = 1;
    	if (turnAngle.isFlip()) {
    		scale = -scale;
    	}
    	return scale;
    }
    
    public static void normalize(VelocityPolar velocities[]) {
    	double maxSpeed = 0;
    	for (int iiWheel = 0; iiWheel < velocities.length; iiWheel++) {
    		if (Math.abs(velocities[iiWheel].speed) > maxSpeed) {
    			maxSpeed = velocities[iiWheel].speed;
    		}
    	}
    	
    	if (maxSpeed > 1.0) {
	    	for (int iiWheel = 0; iiWheel < velocities.length; iiWheel++) {
	    		velocities[iiWheel].speed /= maxSpeed;
	    	}
    	}
    }
    
    /**
     * Calculate wheel velocity change (delta) based on current data.
     * @param rawVelocity Raw wheel change data
     * @return wheel change data (delta) based on current wheel values
     */
    public static VelocityPolar calculateDeltaWheelData(VelocityPolar currentVelocity, VelocityPolar rawVelocity) {
    	VelocityPolar deltaVelocity = new VelocityPolar(0, 0);
		// Compute turn angle from encoder value (pidGet) and raw target value
		AngleFlip turnAngle = computeTurnAngle(currentVelocity.angle, rawVelocity.angle);
		double targetAngle = normalizeAngle(currentVelocity.angle + turnAngle.getAngle()); 
        deltaVelocity.angle = targetAngle;
        deltaVelocity.speed = driveScale(turnAngle) * rawVelocity.speed;
    	return deltaVelocity;
    }
    
}
