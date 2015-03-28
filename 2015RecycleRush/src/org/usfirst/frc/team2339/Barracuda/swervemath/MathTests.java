package org.usfirst.frc.team2339.Barracuda.swervemath;

import static org.junit.Assert.*;

import org.junit.Test;
import org.usfirst.frc.team2339.Barracuda.swervemath.WorkingPalmdale.WheelData;

public class MathTests {

	@Test
	public void testBasicRotate() {
		System.out.println("Hello test world");
		System.out.println("Normalize angle " + 455 + " is " + WorkingPalmdale.normalizeAngle(455));
		
		/*
		 * 45 degree angle case
		 */
		WorkingPalmdale.WHEEL_BASE_LENGTH = 25;
		WorkingPalmdale.WHEEL_BASE_WIDTH = 25;
		WheelData rawWheelData = WorkingPalmdale.calculateRawWheelData(0.0, 0.0, 1.0);
		WheelData rawWheelDataGen = WorkingPalmdale.calculateRawWheelDataGeneral(0.0, 0.0, 1.0, 0.0, 0.0);
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData.wheelSpeeds[ii] +
					", angle " + rawWheelData.wheelAngles[ii]);
			System.out.println("Wheel " + ii + ": Gen speed " + rawWheelDataGen.wheelSpeeds[ii] +
					", angle " + rawWheelDataGen.wheelAngles[ii]);
			assertEquals(rawWheelDataGen.wheelSpeeds[ii], rawWheelData.wheelSpeeds[ii], 0.01);
			assertEquals(rawWheelDataGen.wheelAngles[ii], rawWheelData.wheelAngles[ii], 0.01);
		}

		rawWheelData = WorkingPalmdale.calculateRawWheelData(1.0, 0.0, 1.0);
		rawWheelDataGen = WorkingPalmdale.calculateRawWheelDataGeneral(1.0, 0.0, 1.0, 0.0, 0.0);
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData.wheelSpeeds[ii] +
					", angle " + rawWheelData.wheelAngles[ii]);
			System.out.println("Wheel " + ii + ": Gen speed " + rawWheelDataGen.wheelSpeeds[ii] +
					", angle " + rawWheelDataGen.wheelAngles[ii]);
			assertEquals(rawWheelDataGen.wheelSpeeds[ii], rawWheelData.wheelSpeeds[ii], 0.01);
			assertEquals(rawWheelDataGen.wheelAngles[ii], rawWheelData.wheelAngles[ii], 0.01);
		}
		
		rawWheelData = WorkingPalmdale.calculateRawWheelData(0.0, 1.0, 1.0);
		rawWheelDataGen = WorkingPalmdale.calculateRawWheelDataGeneral(0.0, 1.0, 1.0, 0.0, 0.0);
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData.wheelSpeeds[ii] +
					", angle " + rawWheelData.wheelAngles[ii]);
			System.out.println("Wheel " + ii + ": Gen speed " + rawWheelDataGen.wheelSpeeds[ii] +
					", angle " + rawWheelDataGen.wheelAngles[ii]);
			assertEquals(rawWheelDataGen.wheelSpeeds[ii], rawWheelData.wheelSpeeds[ii], 0.01);
			assertEquals(rawWheelDataGen.wheelAngles[ii], rawWheelData.wheelAngles[ii], 0.01);
		}
		
		/*
		 * Robolope dimensions
		 */
		WorkingPalmdale.WHEEL_BASE_LENGTH = 28.5;
		WorkingPalmdale.WHEEL_BASE_WIDTH = 25;
		rawWheelData = WorkingPalmdale.calculateRawWheelData(0.0, 0.0, 1.0);
		rawWheelDataGen = WorkingPalmdale.calculateRawWheelDataGeneral(0.0, 0.0, 1.0, 0.0, 0.0);
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData.wheelSpeeds[ii] +
					", angle " + rawWheelData.wheelAngles[ii]);
			System.out.println("Wheel " + ii + ": Gen speed " + rawWheelDataGen.wheelSpeeds[ii] +
					", angle " + rawWheelDataGen.wheelAngles[ii]);
			assertEquals(rawWheelDataGen.wheelSpeeds[ii], rawWheelData.wheelSpeeds[ii], 0.01);
			assertEquals(rawWheelDataGen.wheelAngles[ii], rawWheelData.wheelAngles[ii], 0.01);
		}

	}

}
