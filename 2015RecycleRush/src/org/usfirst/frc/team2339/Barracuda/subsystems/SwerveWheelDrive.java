package org.usfirst.frc.team2339.Barracuda.subsystems;

import org.usfirst.frc.team2339.Barracuda.components.SwerveSteeringPidController;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.RectangularCoordinates;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.RobotMotion;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.VelocityPolar;

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
    
    protected VelocityPolar currentVelocity;
    
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
        this.currentVelocity = new VelocityPolar(0, 0);
    	setupMotorSafety();
    }

    public RectangularCoordinates getWheelPosition() {
		return wheelPosition;
	}

	public void setWheelPosition(RectangularCoordinates wheelPosition) {
		this.wheelPosition = wheelPosition;
	}
	
	public VelocityPolar getCurrentVelocity() {
		return currentVelocity;
	}

	private void setCurrentSpeed(double speed) {
		this.currentVelocity.speed = speed;
	}

	private void setCurrentAngle(double angle) {
		this.currentVelocity.angle = angle;
	}

	public double getRadialAngle() {
		return SwerveWheel.getRadialAngle(wheelPosition);
	}
	
	public double getPerpendicularAngle() {
		return SwerveWheel.getPerpendicularAngle(wheelPosition);
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
    	return SwerveWheel.calculateWheelVelocity(wheelNumber, wheelPosition, 
    			pivot, maxWheelRadius, robotMotion);
    }
    
    public void setWheelSpeed(double speed) {
        driveController.set(speed);
        setCurrentSpeed(speed);
        if (safetyHelper != null) safetyHelper.feed();
    }
    
    /**
     * Update speed controller and motor safety using current speed
     */
    public void maintainWheelSpeed(double speed) {
        driveController.set(getCurrentVelocity().speed);
        if (safetyHelper != null) safetyHelper.feed();
    }
    
    public void setSteeringAngle(double angle) {
		SmartDashboard.putNumber("Wheel " + wheelNumber + " set angle ", angle);
        steeringController.setSetpoint(angle);
        setCurrentAngle(angle);
    }
    
    /**
     * Update steering controller and motor safety using current angle
     */
    public void maintainSteeringAngle() {
        steeringController.setSetpoint(getCurrentVelocity().angle);
    }
    
    public double getSteeringAngle() {
        return steeringController.getSteeringAngle();
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
    	setWheel(SwerveWheel.calculateDeltaWheelData(
    			new VelocityPolar(0.0, steeringController.getSteeringAngle()), 
    			velocity));
    	//setWheel(velocity);
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
