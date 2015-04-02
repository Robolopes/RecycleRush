/**
 * 
 */
package org.usfirst.frc.team2339.Barracuda.smartdashboard;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Show a time with a velocity.
 * Good for auto timed drive data.
 * 
 * @author RobolopesDev
 *
 */
public class SendableTimeVelocity implements Sendable {
	
    private ITable table;
    private double time;
    private SendableVelocity velocity;
    
    public SendableTimeVelocity(double time, SendableVelocity velocity) {
		this.time = time;
		this.velocity = velocity;
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#initTable(edu.wpi.first.wpilibj.tables.ITable)
	 */
	@Override
	public void initTable(ITable subtable) {
        this.table = subtable;
        if (table != null) {
            table.putNumber("time", time);
            table.putValue("velocity", velocity);
        }
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getTable()
	 */
	@Override
	public ITable getTable() {
		return table;
	}

	public double getTime() {
		if (getTable() != null) {
			return getTable().getNumber("time");
		}
		return 0;
	}

	public SendableVelocity getVelocity() {
		if (getTable() != null) {
			return (SendableVelocity)getTable().getValue("velocity");
		}
		return null;
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getSmartDashboardType()
	 */
	@Override
	public String getSmartDashboardType() {
		return "Time Velocity";
	}

}
