package org.usfirst.frc.team2339.Barracuda.subsystems;

import org.usfirst.frc.team2339.Barracuda.components.SwerveSteeringPidController;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class to provide swerve drive control to one wheel.
 * Wheel must have independent drive motor and steering motor.
 * A steering encoder provides feedback on wheel angle.
 *   
 * @author emiller
 *
 */
public class SwerveWheelDrive implements MotorSafety {
	
    protected MotorSafetyHelper safetyHelper;
    
    protected int wheelNumber;
    protected RectangularCoordinates wheelPosition;
    
	protected SpeedController driveController;
    protected SwerveSteeringPidController steeringController;
    
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
    	
    	RectangularCoordinates(double x, double y) {
    		this.x = x;
    		this.y = y;
    	}
    	
    	public RectangularCoordinates subtract(RectangularCoordinates p0) {
    		return new RectangularCoordinates(x - p0.x, y - p0.y);
    	}
    	
    	public RectangularCoordinates divide(double r) {
    		return new RectangularCoordinates(x/r, y/r);
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
    	
    	RobotMotion(double strafe, double frontBack, double rotate) {
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
    protected static class AngleFlip {
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
     * Construct swerve drive for a single wheel.
     * 
     * @param wheelNumber wheel number on robot. For information only.
     * @param wheelPosition position of wheel relative to robot.
     * @param driveController speed controller for wheel
     * @param steeringController swerve steering PID controller for this wheel 
     */
    SwerveWheelDrive(
    		int wheelNumber, 
    		RectangularCoordinates wheelPosition, 
    		SpeedController driveController, 
    		SwerveSteeringPidController steeringController) {
        this.wheelNumber = wheelNumber;
        this.wheelPosition = wheelPosition;
        this.driveController = driveController;
        this.steeringController = steeringController;
    	setupMotorSafety();
    }

    public RectangularCoordinates getWheelPosition() {
		return wheelPosition;
	}

	public void setWheelPosition(RectangularCoordinates wheelPosition) {
		this.wheelPosition = wheelPosition;
	}
	
	public double getRadialAngle() {
		return Math.toDegrees(-Math.atan2(wheelPosition.y, wheelPosition.x));
	}
	
	public double getPerpendicularAngle() {
		return Math.toDegrees(Math.atan2(wheelPosition.x, wheelPosition.y));
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
    public VelocityPolar calculateWheelVelocity(
    		RectangularCoordinates wheelPosition,
    		RectangularCoordinates pivot,
    		double maxWheelRadius, 
    		RobotMotion robotMotion) {
    	
    	RectangularCoordinates wheelRelativePosition = wheelPosition.subtract(pivot).divide(maxWheelRadius);
    	RectangularCoordinates wheel = new RectangularCoordinates(
    			robotMotion.strafe + robotMotion.rotate * wheelRelativePosition.y,  
    			robotMotion.frontBack - robotMotion.rotate * wheelRelativePosition.x);
        
        /*
         * Note for angle: atan2(xWheel, yWheel) gives angle in robot coordinates
         * However, wheel has positive counter-clockwise angle and zero is on Y axis.
         * atan2(-yWheel, xWheel) converts to wheel angle system.
         */
        return new VelocityPolar(
        		Math.hypot(wheel.x, wheel.y), 
        		Math.toDegrees(Math.atan2(-wheel.y, wheel.x)));
    }
    
    /**
     * Calculate wheel velocity vector given wheel position relative to pivot location and 
     * desired robot forward, strafe, and rotational velocities.
     * Wheel speed are normalized to the range [0, 1.0]. Angles are normalized to the range [-180, 180).
     * @see https://docs.google.com/presentation/d/1J_BajlhCQ236HaSxthEFL2PxywlneCuLNn276MWmdiY/edit?usp=sharing
     * 
     * @param pivot Position of pivot. 
     * @param maxWheelRadius distance of furtherest wheel on robot from pivot.
     * @param robotMotion desired motion of robot express by strafe, frontBack, and rotation around a pivot point.
     * @return wheel polar velocity (speed and angle)
     */
    public VelocityPolar calculateWheelVelocity(
    		RectangularCoordinates pivot,
    		double maxWheelRadius, 
    		RobotMotion robotMotion) {
    	return calculateWheelVelocity(wheelPosition, pivot, maxWheelRadius, robotMotion);
    }
    
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
    
    /**
     * Calculate wheel velocity change (delta) based on current data.
     * @param rawVelocity Raw wheel change data
     * @return wheel change data (delta) based on current wheel values
     */
    public VelocityPolar calculateDeltaWheelData(VelocityPolar rawVelocity) {
    	VelocityPolar deltaVelocity = new VelocityPolar(0, 0);
		// Compute turn angle from encoder value (pidGet) and raw target value
		SmartDashboard.putNumber("Wheel " + wheelNumber + " current angle ", steeringController.get());
		AngleFlip turnAngle = computeTurnAngle(steeringController.get(), rawVelocity.angle);
		double targetAngle = normalizeAngle(steeringController.get() + turnAngle.getAngle()); 
        deltaVelocity.angle = targetAngle;
        deltaVelocity.speed = driveScale(turnAngle) * rawVelocity.speed;
    	return deltaVelocity;
    }
    
    public void setWheelSpeed(double speed) {
        driveController.set(speed);
        if (safetyHelper != null) safetyHelper.feed();
    }
    
    public void setSteeringAngle(double angle) {
        steeringController.setSetpoint(angle);
    }
    
    public void setWheel(VelocityPolar velocity) {
        setWheelSpeed(velocity.speed);
        setSteeringAngle(velocity.angle);
    }
    
    /**
     * Set wheel in a sane manner. I.e. take into account current angle to move gently shortest angle 
     * and w/o applying speed too quickly
     * @param velocity desired wheel velocity 
     */
    public void setWheelSanely(VelocityPolar velocity) {
    	setWheel(calculateDeltaWheelData(velocity));
    }
    
    public void resetSteering() {
    	boolean isEnabled = steeringController.isEnable();
    	steeringController.reset();
    	enableSteering(isEnabled);
    }
    
    public void enableSteering(boolean enable) {
    	if (enable) {
    		steeringController.enable();
    	} else {
    		steeringController.disable();
    	}
    }
    
    private void setupMotorSafety() {
        safetyHelper = new MotorSafetyHelper(this);
        setExpiration(MotorSafety.DEFAULT_SAFETY_EXPIRATION);
        setSafetyEnabled(true);
    }

	@Override
	public void setExpiration(double timeout) {
        safetyHelper.setExpiration(timeout);
	}

	@Override
	public double getExpiration() {
        return safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
        return safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		driveController.set(0.0);
        if (safetyHelper != null) safetyHelper.feed();
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
        safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
        return safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		return "Swerve wheel drive";
	}

}
