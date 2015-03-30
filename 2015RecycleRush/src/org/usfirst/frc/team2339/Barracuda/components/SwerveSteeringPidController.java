package org.usfirst.frc.team2339.Barracuda.components;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

/**
 * PID controller for steering swerve drive wheel
 * 
 * @author emiller
 *
 */
public class SwerveSteeringPidController extends PIDController {

	private final SwerveSteeringEncoder steeringEncoder;
	
	public SwerveSteeringPidController(double Kp, double Ki, double Kd,
			SwerveSteeringEncoder steeringEncoder, SpeedController steeringController) {
		super(Kp, Ki, Kd, steeringEncoder, steeringController);
		this.steeringEncoder = steeringEncoder;
        this.setInputRange(-180, 180);
        this.setContinuous(true);
	}
	
	public double getSteeringAngle() {
		return steeringEncoder.getDistance();
	}

}
