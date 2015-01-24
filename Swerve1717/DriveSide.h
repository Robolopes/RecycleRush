/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#ifndef DRIVE_SIDE_H_
#define DRIVE_SIDE_H_

#include "WPILib.h"
#include "../../SteeringMotor.h"

/**
 * Each side of our drive train has a Jaguar/Encoder pair for driving,
 * and a SteeringMotor (w/ a potentiometer) for steering.
 */
class DriveSide
{
	friend class DriveTrainManager;
	public:
		/**
		 * Constructs a new DriveSide
		 * @param DriveJaguar The information about the Jaguar to be used for driving.
		 * @param EncoderA The information for the Encoder's 'A' port (B is presumed to follow right after).
		 * @param PotInfo The information for the Potentiometer.
		 * @param potCenterVoltage The voltage the pot should have when centered forwards.
		 * @param voltsPerDegree The number of volts that represents a change of one degree (ex.: 5 / 360.f).  Can be inverted if the motor corrects in the wrong direction.
		 * @param steeringMotorFriction The friction required to move the SteeringMotor.  This is pretty much BS.
		 */
		DriveSide(UINT32 DriveJaguar, UINT32 EncoderA, UINT32 PotInfo, UINT32 SteeringMotorInfo, float potCenterVoltage, float voltsPerDegree, PIDValues controlConstants);
		virtual ~DriveSide(){}
		
		/**
		 * Set the driving speed.  
		 * @param speed The speed to set it to.
		 */
		void Set(float speed);
		
		/**
		 * Return the actual value passed to the driving Jaguar most recently.
		 * Note: This may be radically different from the last value passed to
		 * Set, because of traction control and other driving modes.
		 */
		float Get(void);	
		
		/**
		 * Set the friction constant without reconstructing the DriveSide.
		 * @param friction The new motor friction for the SteeringMotor to have.
		 */
		void SetDriveConstants(float friction);
		
		/**
		 * Turn the SteeringMotor to an angle.
		 * @param angle The angle to rotate to.  Angles are relative to the positive y-axis (forward), with clockwise being positive.  Wrapping OK.
		 */
		void TurnTo(float angle);
		
		/**
		 * Frequently called method to update the SteeringMotor control loop and,
		 * (sometimes) update the traction control loop.
		 */
		void Update();
		
		/**
		 * Debug method to printf info about the SteeringMotor.
		 */
		void PrintInfo();
		
		/**
		 * Get the speed of the drive using encoders.
		 * @return Speed in meters per second.
		 */
		float GetSpeed();
		
		float GetGoalAngle() { return mSteeringMotor->GetGoalAngle(); }
		float GetAngle() { return mSteeringMotor->GetCurrentAngle(); }
		

		
	private:
		
		/**
		 * Counts time.
		 * @return us elapsed since last call to this method.
		 */
		float _getPeriod();
		
		/**
		 * Counts ticks.
		 * @return Ticks encountered since last call to this method.
		 */
		INT32 _getDeltaTicks();
		
		/**
		 * Calls _getDeltaTicks() and converts to engineering units.
		 * @return Meters travelled.
		 */
		float _getDistance();
		
		float _getSpeed();
		
		float				mLastVelocity;
		Jaguar* 			mDriveJag;
		SteeringMotor* 		mSteeringMotor;
		Encoder* 			mDriveEncoder;
		double				mDriveErrorCorrectionConstant;
		float				mLastSpeed;
		UINT32				mPreviousTimestamp;
		float				mDriveMotorFriction;
		UINT32				mPrevStamp;
		INT32				mPrevTicks;
		float				mPrevSpeed;
};

#endif
