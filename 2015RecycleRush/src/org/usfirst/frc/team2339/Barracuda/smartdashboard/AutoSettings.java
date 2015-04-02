/**
 * 
 */
package org.usfirst.frc.team2339.Barracuda.smartdashboard;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Display choices and accept input for Robolopes autonomous
 * 
 * @author RobolopesDev
 *
 */
public class AutoSettings implements Sendable {
	
    private ITable table;
    private final SendableTimeVelocity forward;
    private final SendableTimeSpeed lift;
    private final SendableTimeVelocity back;
    
    public AutoSettings() {
    	this.forward = new SendableTimeVelocity(0, new SendableVelocity(0, 0));
    	this.lift = new SendableTimeSpeed(0, 0);
    	this.back = new SendableTimeVelocity(0, new SendableVelocity(0, 0));
    }

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#initTable(edu.wpi.first.wpilibj.tables.ITable)
	 */
	@Override
	public void initTable(ITable subtable) {
        this.table = subtable;
        if (table != null) {
            table.putValue("forward", forward);
            table.putValue("lift", lift);
            table.putValue("back", back);
        }
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getTable()
	 */
	@Override
	public ITable getTable() {
		return table;
	}

	public SendableTimeVelocity getForward() {
		if (getTable() != null) {
			return (SendableTimeVelocity)getTable().getValue("forward");
		}
		return null;
	}

	public SendableTimeSpeed getLift() {
		if (getTable() != null) {
			return (SendableTimeSpeed)getTable().getValue("lift");
		}
		return null;
	}

	public SendableTimeVelocity getBack() {
		if (getTable() != null) {
			return (SendableTimeVelocity)getTable().getValue("back");
		}
		return null;
	}

	/* (non-Javadoc)
	 * @see edu.wpi.first.wpilibj.Sendable#getSmartDashboardType()
	 */
	@Override
	public String getSmartDashboardType() {
		return "Robolopes Autonomous Settings";
	}

}
