#ifndef STEERING_MOTOR_H_
#define STEERING_MOTOR_H_

#include "WPILib.h"
#include "sim_pid/SimPID.h"

class SteeringMotor
{
	public:
		SteeringMotor(UINT32 jaguarSlotChannel,
								UINT32 potSlotChannel,
								float maxSpeed=1.0,
								bool wrapAround=true,
								float negLim = -361,
								float posLim = 361);
		
		void	SetControlConstants(float voltsPerDegree,
				float centerVoltage = 2.5f, PIDValues controlConstants = PIDValues());
		void 	GoToAngle(float angle);
		void	GoToRelativeAngle(float angle);
		void	Update();
		void	SetSpeed(float speed = 0.0f);
		float 	GetCurrentAngle();
		float	GetGoalAngle() { return mGoalAngle; }
		//call V frequently
		float	GetDegreesPerSecond();
		void	PrintInfo();
		void	GoToAngleWithoutUpdating(float angle);
		
	private:
		
		float	_getUnfilteredAngle();
		float	_getDistance(float angle1, float angle2);
		
		float 				mCurrentAngle;
		float				mGoalAngle;
		float				mCenterVoltage;
		float				mVoltsPerDegree;
		float				mLastValue;
		float				mLastTime;
		float				mLastVelocity;
		float				mNegLim;
		float				mPosLim;
		SpeedController* 	mSpeedController;
		AnalogChannel*		mAnalogChannel;
		SimPID*				mLoopControl;
		float				mLastAngle;
		UINT32				mPotInfo;
		
};

#endif /* STEERING_MOTOR_H_ */
