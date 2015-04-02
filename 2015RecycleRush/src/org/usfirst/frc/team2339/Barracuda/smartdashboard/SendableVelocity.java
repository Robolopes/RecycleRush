/**
 * 
 */
package org.usfirst.frc.team2339.Barracuda.smartdashboard;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Show velocity (speed and angle) data
 * 
 * @author RobolopesDev
 *
 */
public class SendableVelocity implements Sendable {
	
    private ITable table;
    private double speed;
    private double angle;
    
    public SendableVelocity(double speed, double angle) {
		this.speed = speed;
		this.angle = angle;
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#initTable(edu.wpi.first.wpilibj.tables.ITable)
	 */
	@Override
	public void initTable(ITable subtable) {
        this.table = subtable;
        if (table != null) {
            table.putNumber("speed", speed);
            table.putNumber("angle", angle);
        }
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getTable()
	 */
	@Override
	public ITable getTable() {
		return table;
	}
	
	public double getSpeed() {
		if (getTable() != null) {
			return getTable().getNumber("speed");
		}
		return 0;
	}

	public double getAngle() {
		if (getTable() != null) {
			return getTable().getNumber("angle");
		}
		return 0;
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getSmartDashboardType()
	 */
	@Override
	public String getSmartDashboardType() {
		return "Velocity";
	}

}
