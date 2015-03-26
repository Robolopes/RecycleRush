package org.usfirst.frc.team2339.Barracuda.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Lift extends Subsystem {

    private final Talon liftMotor;

	public Lift(int liftMotorNumber) {
		this.liftMotor = new Talon(liftMotorNumber);
	}
    
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    /**
     * Set the lift motor
     * 
     * @param value motor speed
     */
    public void setLiftMotor(double value) {
        SmartDashboard.putNumber("Lift motor value ", value);
        liftMotor.set(value);
    }
    
    /**
     * Stop the lift motor
     * 
     */
    public void stopLiftMotor() {
        setLiftMotor(0.0);
    }
    
    
}

