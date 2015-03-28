package org.usfirst.frc.team2339.Barracuda.swervemath;

import static org.junit.Assert.*;

import org.junit.Test;
import org.usfirst.frc.team2339.Barracuda.swervemath.WorkingPalmdale.WheelData;

public class MathTests {

	@Test
	public void test() {
		System.out.println("Hello test world");
		System.out.println("Normalize angle " + 455 + " is " + WorkingPalmdale.normalizeAngle(455));
		
		WheelData rawWheelData = WorkingPalmdale.calculateRawWheelData(0.0, 0.0, 1.0);
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData.wheelSpeeds[ii] +
					", angle " + rawWheelData.wheelAngles[ii]);
		}

		WorkingPalmdale.WHEEL_BASE_LENGTH = 25;
		WorkingPalmdale.WHEEL_BASE_WIDTH = 25;
		rawWheelData = WorkingPalmdale.calculateRawWheelData(0.0, 0.0, 1.0);
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData.wheelSpeeds[ii] +
					", angle " + rawWheelData.wheelAngles[ii]);
		}
	}

}
