/**
 * 
 */
package org.usfirst.frc.team2339.Barracuda.smartdashboard;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Show time and speed.
 * Good for auto time to run a motor, like a lift motor.
 * 
 * @author RobolopesDev
 *
 */
public class SendableTimeSpeed implements Sendable {
	
    private ITable table;
    private double time;
    private double speed;
    
    public SendableTimeSpeed(double time, double speed) {
		this.time = time;
		this.speed = speed;
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#initTable(edu.wpi.first.wpilibj.tables.ITable)
	 */
	@Override
	public void initTable(ITable subtable) {
        this.table = subtable;
        if (table != null) {
            table.putValue("time", time);
            table.putValue("speed", speed);
        }
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getTable()
	 */
	@Override
	public ITable getTable() {
		return table;
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getSmartDashboardType()
	 */
	@Override
	public String getSmartDashboardType() {
		return "Time Speed";
	}

}
