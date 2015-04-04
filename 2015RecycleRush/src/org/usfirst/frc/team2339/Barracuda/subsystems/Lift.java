package org.usfirst.frc.team2339.Barracuda.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Lift extends Subsystem {

    private final Talon liftMotor;
    private final DigitalInput lowerLimitSwitch;
	public Lift(int liftMotorNumber, int lowerLimitSwitchChannel) {
		this.liftMotor = new Talon(liftMotorNumber);
		this.lowerLimitSwitch = new DigitalInput(lowerLimitSwitchChannel);
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
        if (!lowerLimitSwitch.get() && value < 0.0) {
            SmartDashboard.putNumber("Lift motor value ", 0);
        	liftMotor.set(0.0);
        } else {
            SmartDashboard.putNumber("Lift motor value ", value);
        	liftMotor.set(value);
        }
    }
    
    /**
     * Stop the lift motor
     * 
     */
    public void stopLiftMotor() {
        setLiftMotor(0.0);
    }
    
    
}

