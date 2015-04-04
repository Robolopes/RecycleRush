/**
 * 
 */
package org.usfirst.frc.team2339.robot;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Show position data (x,y). Also good for showing x,y vector
 * 
 * @author RobolopesDev
 *
 */
public class SendablePosition implements Sendable {
	
    private ITable table;
    private double x;
    private double y;
    
    public SendablePosition(double x, double y) {
		this.x = x;
		this.y = y;
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#initTable(edu.wpi.first.wpilibj.tables.ITable)
	 */
	@Override
	public void initTable(ITable subtable) {
        this.table = subtable;
        if (table != null) {
            table.putNumber("x", x);
            table.putNumber("y", y);
        }
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getTable()
	 */
	@Override
	public ITable getTable() {
		return table;
	}

	public double getX() {
		if (getTable() != null) {
			return getTable().getNumber("x");
		}
		return 0;
	}

	public double getY() {
		if (getTable() != null) {
			return getTable().getNumber("y");
		}
		return 0;
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getSmartDashboardType()
	 */
	@Override
	public String getSmartDashboardType() {
		return "Position";
	}

}
