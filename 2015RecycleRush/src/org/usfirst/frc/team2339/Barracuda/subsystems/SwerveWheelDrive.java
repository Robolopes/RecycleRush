package org.usfirst.frc.team2339.Barracuda.subsystems;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

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
    public static final double kDefaultExpirationTime = MotorSafety.DEFAULT_SAFETY_EXPIRATION;
    
    protected int wheelNumber;
    protected SpeedController driveController;
    protected PIDController steeringController;
    
    SwerveWheelDrive(
    		int wheelNumber, 
    		SpeedController driveController, 
    		PIDController steeringController) {
        this.wheelNumber = wheelNumber;
        this.driveController = driveController;
        this.steeringController = steeringController;
    	setupMotorSafety();
    }

    public void setWheelSpeed(double speed) {
        driveController.set(speed);
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
        this.setExpiration(kDefaultExpirationTime);
        this.setSafetyEnabled(true);
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
