package org.usfirst.frc.team2339.Barracuda.components;

import edu.wpi.first.wpilibj.Encoder;

public class SwerveSteeringEncoder extends Encoder {

	/**
	 * Construct a swerve steering encoder
	 * 
	 * @param aChannel Encoder first channel number
	 * @param bChannel Encoder second channel number
	 * @param degreesPerPulse Angular distance encoder travels between pulses in degrees.
	 */
	public SwerveSteeringEncoder(int aChannel, int bChannel, double degreesPerPulse) {
		super(aChannel, bChannel);
        this.setDistancePerPulse(degreesPerPulse);
	}
}
