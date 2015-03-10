package org.usfirst.frc.team2339.Barracuda.subsystems;

import org.usfirst.frc.team2339.Barracuda.components.SwerveSteeringPidController;
import org.usfirst.frc.team2339.Barracuda.subsystems.SwerveWheelDrive.RectangularCoordinates;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Swerve drive with four wheels in a rectangular configuration.
 * 
 * @author emiller
 *
 */
public class SwerveDriveRectangle extends SwerveDrive {

	/**
	 * 
	 * @param length length of wheel base (front-back)
	 * @param width width of wheel base (side-side)
	 * @param driveControllers
	 */
	public SwerveDriveRectangle(SwerveWheelDrive[] wheels) {
		super(wheels);
	}

	/**
	 * Create four wheels for swerve drive.
	 * Wheels are on corners of a rectangle defined by the wheel base length and width.
	 * Wheel 0 is front right, 1 front left, 2 rear left, 3 rear right. 
	 * @param length length of wheel base (front-back)
	 * @param width width of wheel base (side-side)
	 * @param driveControllers wheel drive controllers. Must be exactly four.
	 * @param steeringControllers steering PID controllers. Must be exactly four.
	 * @return
	 */
	public static SwerveWheelDrive[] createWheels(
			double length, 
			double width,
			SpeedController driveControllers[],
			SwerveSteeringPidController steeringControllers[]) {
		
		SwerveWheelDrive[] wheels = new SwerveWheelDrive[4];
		for (int iiWheel = 0; iiWheel < 4; iiWheel++) {
			double x, y;
			switch(iiWheel) {
			case 0:
			default:
				x = width/2;
				y = length/2;
				break;
			case 1:
				x = -width/2;
				y = length/2;
				break;
			case 2:
				x = -width/2;
				y = -length/2;
				break;
			case 3:
				x = width/2;
				y = -length/2;
				break;
				
			}
			wheels[iiWheel] = new SwerveWheelDrive(
					iiWheel, 
					new RectangularCoordinates(x, y), 
					driveControllers[iiWheel], 
					steeringControllers[iiWheel]);
		}
		return wheels;
	}

}
