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
    protected SpeedController driveController;
    protected SwerveSteeringPidController steeringController;
    
    public class WheelVelocityVector {
    	public double wheelSpeed = 0;
    	public double wheelAngle = 0;
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
    
    SwerveWheelDrive(
    		int wheelNumber, 
    		SpeedController driveController, 
    		SwerveSteeringPidController steeringController) {
        this.wheelNumber = wheelNumber;
        this.driveController = driveController;
        this.steeringController = steeringController;
    	setupMotorSafety();
    }

    /**
     * Calculate wheel velocity vector given wheel position relative to pivot location and 
     * desired robot forward, strafe, and rotational velocities.
     * Wheel speed are normalized to the range [0, 1.0]. Angles are normalized to the range [-180, 180).
     * @see https://docs.google.com/presentation/d/1J_BajlhCQ236HaSxthEFL2PxywlneCuLNn276MWmdiY/edit?usp=sharing
     * @param xWheelPosition distance of wheel to right of pivot (left is negative).
     * @param yWheelPosition distance of wheel in front of pivot (back is negative).
     * @param maxWheelRadius distance of furtherest wheel on robot from pivot.
     * @param xVelocity strafe (sideways) velocity. -1.0 = max motor speed left. 1.0 = max motor speed right.
     * @param yVelocity forward velocity. -1.0 = max motor speed backwards. 1.0 = max motor speed forward.
     * @param rotateVelocity clockwise rotational velocity. -1.0 = max motor speed counter-clockwise. 1.0 = max motor speed clockwise.
     * @return wheel vector (speed and angle)
     */
    public WheelVelocityVector calculateWheelVelocityVector(
    		double xWheelPosition, 
    		double yWheelPosition, 
    		double maxWheelRadius, 
    		double xVelocity, 
    		double yVelocity, 
    		double rotateVelocity) {
        double xWheel = xVelocity + rotateVelocity * yWheelPosition / maxWheelRadius; 
        double yWheel = yVelocity - rotateVelocity * xWheelPosition / maxWheelRadius;
        WheelVelocityVector wheelVelocity = new WheelVelocityVector();
        wheelVelocity.wheelSpeed = Math.hypot(xWheel, yWheel);
        /*
         * atan2(xWheel, yWheel) gives angle in robot coordinates
         * However, wheel has positive counter-clockwise angle and zero is on Y axis.
         * atan2(-yWheel, xWheel) converts to wheel angle system.
         */
        wheelVelocity.wheelAngle = Math.toDegrees(Math.atan2(-yWheel, xWheel));
        return wheelVelocity;
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
     * Calculate wheel data change (delta) based on current data.
     * @param rawWheelData Raw wheel change data
     * @return wheel change data (delta) based on current wheel values
     */
    public WheelVelocityVector calculateDeltaWheelData(WheelVelocityVector rawWheelData) {
    	WheelVelocityVector deltaWheelData = new WheelVelocityVector();
		// Compute turn angle from encoder value (pidGet) and raw target value
		SmartDashboard.putNumber("Wheel " + wheelNumber + " current angle ", steeringController.get());
		AngleFlip turnAngle = computeTurnAngle(steeringController.get(), rawWheelData.wheelAngle);
		double targetAngle = normalizeAngle(steeringController.get() + turnAngle.getAngle()); 
        deltaWheelData.wheelAngle = targetAngle;
        deltaWheelData.wheelSpeed = driveScale(turnAngle) * rawWheelData.wheelSpeed;
    	return deltaWheelData;
    }
    
    public void setWheelSpeed(double speed) {
        driveController.set(speed);
        if (safetyHelper != null) safetyHelper.feed();
    }
    
    public void setSteeringAngle(double angle) {
        steeringController.setSetpoint(angle);
    }
    
    public void resetSteering() {
    	boolean isEnabled = steeringController.isEnable();
    	steeringController.reset();
    	steeringEnable(isEnabled);
    }
    
    public void steeringEnable(boolean enable) {
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
