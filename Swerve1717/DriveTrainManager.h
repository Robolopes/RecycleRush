/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#ifndef DRIVETRAIN_MANAGER_H_
#define DRIVETRAIN_MANAGER_H_

#include "WPILIb.h"
#include "DriveSide/DriveSide.h"


/**
 * This class controls all aspects of the drive train.
 */
class DriveTrainManager
{
	public:
		DriveTrainManager();
		~DriveTrainManager(){}
		
		/**
		 * Debug print method.
		 */
		void PrintSpeed();
		
		void PrintCenterVoltages();

		/**
		 * Process the input from the DS during teleop and
		 * controls the drive train.
		 */
		void ProcessTeleop();
		
	private:
		
		/**
		 * Raw Mode only
		 */
		void _RawMode();
		
		/**
		 * In vector mode the left joystick is the vector we want to go in and the right joystick is skid adjustment.
		 */
		void _processVectorMode();
		
		
		
		/**
		 * Control the right wheel steering.
		 * @param angle The angle to go to.
		 */
		void _setRightAngle(float angle);
			
		/**
		 * Control the left wheel steering.
		 * @param angle The angle to go to.
		 */
		void _setLeftAngle(float angle);
		
		/**
		 * Control the left wheel power.
		 * @param speed The speed to go.
		 */
		void _setLeftSpeed(float speed);
			
		/**
		 * Control the right wheel power.
		 * @param speed The speed to go.
		 */
		void _setRightSpeed(float speed);
		
		
		
		/**
		 * Left DriveSide
		 */
		DriveSide* 		mLeftDrive;
		
		/**
		 * Right DriveSide
		 */
		DriveSide* 		mRightDrive;
		
		
};

#endif
