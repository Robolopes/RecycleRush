package org.usfirst.frc.team2339.Barracuda.subsystems;

import edu.wpi.first.wpilibj.MotorSafety;
import edu.wpi.first.wpilibj.MotorSafetyHelper;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * Class to provide swerve drive control to one wheel.
 * Wheel must have independent drive motor and steering motor.
 * A steering encoder provides feedback on wheel angle.  
 * @author emiller
 *
 */
public class SwerveWheelDrive implements MotorSafety {
	
    protected MotorSafetyHelper m_safetyHelper;
    public static final double kDefaultExpirationTime = MotorSafety.DEFAULT_SAFETY_EXPIRATION;

    protected SpeedController m_driveController;
    
    SwerveWheelDrive() {
    	setupMotorSafety();
    }

    private void setupMotorSafety() {
        m_safetyHelper = new MotorSafetyHelper(this);
        this.setExpiration(kDefaultExpirationTime);
        this.setSafetyEnabled(true);
    }

	@Override
	public void setExpiration(double timeout) {
        m_safetyHelper.setExpiration(timeout);
	}

	@Override
	public double getExpiration() {
        return m_safetyHelper.getExpiration();
	}

	@Override
	public boolean isAlive() {
        return m_safetyHelper.isAlive();
	}

	@Override
	public void stopMotor() {
		m_driveController.set(0.0);
        if (m_safetyHelper != null) m_safetyHelper.feed();
	}

	@Override
	public void setSafetyEnabled(boolean enabled) {
        m_safetyHelper.setSafetyEnabled(enabled);
	}

	@Override
	public boolean isSafetyEnabled() {
        return m_safetyHelper.isSafetyEnabled();
	}

	@Override
	public String getDescription() {
		return "Swerve wheel drive";
	}

}
