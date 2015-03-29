package org.usfirst.frc.team2339.Barracuda.swervemath.test;

import static org.junit.Assert.*;

import org.junit.Test;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.AngleFlip;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.RectangularCoordinates;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.RobotMotion;
import org.usfirst.frc.team2339.Barracuda.swervemath.SwerveWheel.VelocityPolar;
import org.usfirst.frc.team2339.Barracuda.swervemath.WorkingPalmdale;
import org.usfirst.frc.team2339.Barracuda.swervemath.WorkingPalmdale.WheelData;

public class MathTests {
	
	// Merged 2015-03-29

	@Test
	public void testPalmdaleRotate() {
		System.out.println("*********** testPalmdaleRotate start ***********");
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

		System.out.println("*********** testPalmdaleRotate end ***********");
	}

	@Test
	public void testNewRotate() {
		System.out.println("*********** testNewRotate start ***********");
		VelocityPolar[] rawWheelData = SwerveWheel.calculateRectangularWheelVelocities(25, 25, 
				new RobotMotion(0.0, 0.0, 1.0));
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData[ii].speed +
					", angle " + rawWheelData[ii].angle);
		}
		assertEquals(rawWheelData[0].angle, -rawWheelData[3].angle, 0.1);
		assertEquals(rawWheelData[1].angle, -rawWheelData[2].angle, 0.1);
		assertEquals(Math.abs(rawWheelData[0].angle + rawWheelData[1].angle), 180, 0.1);

		rawWheelData = SwerveWheel.calculateRectangularWheelVelocities(25, 25, 
				new RobotMotion(0.0, 0.0, -1.0));
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData[ii].speed +
					", angle " + rawWheelData[ii].angle);
		}
		assertEquals(rawWheelData[0].angle, -rawWheelData[3].angle, 0.1);
		assertEquals(rawWheelData[1].angle, -rawWheelData[2].angle, 0.1);
		assertEquals(Math.abs(rawWheelData[0].angle + rawWheelData[1].angle), 180, 0.1);

		rawWheelData = SwerveWheel.calculateRectangularWheelVelocities(28.5, 25, 
				new RobotMotion(0.0, 0.0, 1.0));
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData[ii].speed +
					", angle " + rawWheelData[ii].angle);
		}
		assertEquals(rawWheelData[0].angle, -rawWheelData[3].angle, 0.1);
		assertEquals(rawWheelData[1].angle, -rawWheelData[2].angle, 0.1);
		assertEquals(Math.abs(rawWheelData[0].angle + rawWheelData[1].angle), 180, 0.1);

		System.out.println("*********** testNewRotate end ***********");
	}
	
	@Test
	public void testContainerRotate() {
		System.out.println("*********** testContainerRotate start ***********");
		RectangularCoordinates containerPivot = new RectangularCoordinates(0, 2 + 28.5/2);
		VelocityPolar[] rawWheelData = SwerveWheel.calculateRectangularWheelVelocities(28.5, 25, 
				new RobotMotion(0.0, 0.0, 1.0), containerPivot);
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData[ii].speed +
					", angle " + rawWheelData[ii].angle);
		}
		assertEquals(Math.abs(rawWheelData[0].angle + rawWheelData[1].angle), 180, 0.1);
		assertEquals(Math.abs(rawWheelData[2].angle + rawWheelData[3].angle), 180, 0.1);

		rawWheelData = SwerveWheel.calculateRectangularWheelVelocities(28.5, 25, 
				new RobotMotion(0.0, 0.0, -1.0), containerPivot);
		for (int ii = 0; ii < WorkingPalmdale.kMaxNumberOfMotors; ii++) {
			System.out.println("Wheel " + ii + ": speed " + rawWheelData[ii].speed +
					", angle " + rawWheelData[ii].angle);
		}
		assertEquals(Math.abs(rawWheelData[0].angle + rawWheelData[1].angle), 180, 0.1);
		assertEquals(Math.abs(rawWheelData[2].angle + rawWheelData[3].angle), 180, 0.1);

		System.out.println("*********** testContainerRotate end ***********");
	}
	
	@Test
	public void testDelta() {
		System.out.println("*********** testDelta start ***********");
		VelocityPolar deltaVel = SwerveWheel.calculateDeltaWheelData(
				new VelocityPolar(0, 0), 
				new VelocityPolar(1, 180));
		assertEquals(-1.0, deltaVel.speed, 0.1);
		assertEquals(0, deltaVel.angle, 0.1);
		deltaVel = SwerveWheel.calculateDeltaWheelData(
				new VelocityPolar(0, 90), 
				new VelocityPolar(1, -90));
		assertEquals(-1.0, deltaVel.speed, 0.1);
		assertEquals(90, deltaVel.angle, 0.1);
		deltaVel = SwerveWheel.calculateDeltaWheelData(
				new VelocityPolar(0, 0), 
				new VelocityPolar(1, 45));
		assertEquals(1.0, deltaVel.speed, 0.1);
		assertEquals(45, deltaVel.angle, 0.1);
		deltaVel = SwerveWheel.calculateDeltaWheelData(
				new VelocityPolar(0, 0), 
				new VelocityPolar(1, 135));
		assertEquals(-1.0, deltaVel.speed, 0.1);
		assertEquals(-45, deltaVel.angle, 0.1);
		deltaVel = SwerveWheel.calculateDeltaWheelData(
				new VelocityPolar(0, 0), 
				new VelocityPolar(1, 10));
		assertEquals(1.0, deltaVel.speed, 0.1);
		assertEquals(10, deltaVel.angle, 0.1);
		deltaVel = SwerveWheel.calculateDeltaWheelData(
				new VelocityPolar(0, 0), 
				new VelocityPolar(1, -10));
		assertEquals(1.0, deltaVel.speed, 0.1);
		assertEquals(-10, deltaVel.angle, 0.1);
		deltaVel = SwerveWheel.calculateDeltaWheelData(
				new VelocityPolar(0, 0), 
				new VelocityPolar(1, 170));
		assertEquals(-1.0, deltaVel.speed, 0.1);
		assertEquals(-10, deltaVel.angle, 0.1);
		deltaVel = SwerveWheel.calculateDeltaWheelData(
				new VelocityPolar(0, 0), 
				new VelocityPolar(1, -170));
		assertEquals(-1.0, deltaVel.speed, 0.1);
		assertEquals(10, deltaVel.angle, 0.1);
		System.out.println("*********** testDelta end ***********");
	}

	@Test
	public void testTurnAngle() {
		System.out.println("*********** testTurnAngle start ***********");
		AngleFlip turnAngle = SwerveWheel.computeTurnAngle(0, 45);
		assertEquals(45, turnAngle.getAngle(), 0.1);
		assertFalse(turnAngle.isFlip());
		turnAngle = SwerveWheel.computeTurnAngle(0, 135);
		assertEquals(-45, turnAngle.getAngle(), 0.1);
		assertTrue(turnAngle.isFlip());
		turnAngle = SwerveWheel.computeTurnAngle(-90, 90);
		assertEquals(0, turnAngle.getAngle(), 0.1);
		assertTrue(turnAngle.isFlip());
		System.out.println("*********** testTurnAngle end ***********");
	}
}
