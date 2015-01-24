/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#include "DriveSide.h"
#include "../../PenguinLib/PenguinLib.h"


const float WHEEL_CIRCUMFERENCE = 6.09 * PI * 2.54 / 100;
const float ACC_MAX = 1.000; // m / s^2
const float TIME_TO_CROSS_FIELD = 6.3; // s
const float MAX_CIM_RPM = 5500; //rpm
const float DRIVE_GEAR_RATIO = 5.854;//:1
const float RPM_AT_NORMAL_VOLTAGE = MAX_CIM_RPM / DRIVE_GEAR_RATIO; //rpm
const float RPS_AT_NORMAL_VOLTAGE = RPM_AT_NORMAL_VOLTAGE / 60; //rps
const float SPEED_AT_NORMAL_VOLTAGE = RPS_AT_NORMAL_VOLTAGE * WHEEL_CIRCUMFERENCE; //m/s
const float MAX_CIM_SPEED = MAX_CIM_RPM * WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;// m/s
const float CIM_MOTOR_FRICTION = 0.20f;
const UINT32 PULSES_PER_REVOLUTION = 250;

DriveSide::DriveSide(UINT32 DriveJaguar, UINT32 EncoderA, UINT32 PotInfo, UINT32 SteeringMotorInfo, float potCenterVoltage, float voltsPerDegree, PIDValues controlConstants)
{
	mDriveJag = new Jaguar(PenguinLib::VirtualSlot(DriveJaguar), PenguinLib::VirtualChannel(DriveJaguar));
	mDriveEncoder = new Encoder(PenguinLib::VirtualSlot(EncoderA), PenguinLib::VirtualChannel(EncoderA), PenguinLib::VirtualSlot(EncoderA), PenguinLib::VirtualChannel(EncoderA) + 1);
	mDriveEncoder->Start();
	
	mDriveErrorCorrectionConstant = 1.0f;
	mSteeringMotor = new SteeringMotor(SteeringMotorInfo, PotInfo);
	mSteeringMotor->SetControlConstants(voltsPerDegree, potCenterVoltage, controlConstants);
	mLastSpeed = 0.0;
	mDriveMotorFriction = CIM_MOTOR_FRICTION;
	mPreviousTimestamp = GetFPGATime();
	
	mPrevTicks = mDriveEncoder->Get();
	mPrevSpeed = 0.0;
	
	//initialize change variables
	_getDeltaTicks();
	_getPeriod();
	
	mLastVelocity = 0.0;
}

void DriveSide::SetDriveConstants(float friction)
{
	mDriveMotorFriction = friction;
}

void DriveSide::TurnTo(float angle)
{
	mSteeringMotor->GoToAngle(angle);
}
void DriveSide::Set(float speed)
{
	UINT32 temp = GetFPGATime();
	
	if((speed < 0 && mLastSpeed > 0) || (speed > 0 && mLastSpeed < 0))
		speed = 0;
	

		
	
	/*if(mTractionControl == TractionStateOn && speed != 0)
	{
		UINT32 delta_t = temp - mPreviousTimestamp; //this is shitty
		//float increment = ACC_MAX / RPS_AT_NORMAL_VOLTAGE * delta_t / 1000000.f;
		float step = ACC_MAX / SPEED_AT_NORMAL_VOLTAGE * delta_t / 1000000.f;
		//float increment = 0.001;
		if (speed > mLastSpeed)
		{
			if (speed -mLastSpeed > step)
				speed = mLastSpeed + step;
		} 
		else if (speed < mLastSpeed)
		{
			if (mLastSpeed - speed > step)
				speed = mLastSpeed - step;
		}
	}*/
	mLastSpeed = speed;
	speed = speed * mDriveErrorCorrectionConstant;
	if (speed > 0)
	{
		//speed = (speed / (1 - mDriveMotorFriction)) + mDriveMotorFriction;
	}
	else if (speed < 0)
	{
		//speed = (speed / (1 - mDriveMotorFriction)) - mDriveMotorFriction;
	}
	
	mDriveJag->Set(speed);
	
	mPreviousTimestamp = temp;
}

float DriveSide::Get(void)
{
	return mDriveJag->Get();
}

void DriveSide::Update()
{
	mSteeringMotor->Update();
	GetSpeed(); //refresh delays etc
}

void DriveSide::PrintInfo()
{
	mSteeringMotor->PrintInfo();
}

float DriveSide::_getPeriod()
{	
	UINT32 now = GetFPGATime();
	UINT32 delta = now - mPrevStamp;
	mPrevStamp = now;
	return (float)delta / 1000000.f;
}

INT32 DriveSide::_getDeltaTicks()
{
	INT32 ticks = mDriveEncoder->Get();
	INT32 delta = ticks - mPrevTicks;
	mPrevTicks = ticks;
	return delta;
}

float DriveSide::GetSpeed()
{
	return mLastVelocity;
}

// meters per second
float DriveSide::_getSpeed()
{
	float speed = 60 * _getDistance() / _getPeriod();
	mLastVelocity = speed;
	return speed;
}

//revolutions
float DriveSide::_getDistance()
{
	float distance = (float)_getDeltaTicks() / PULSES_PER_REVOLUTION;  //(_getDeltaTicks() / (PULSES_PER_REVOLUTION * DRIVE_GEAR_RATIO)) * WHEEL_CIRCUMFERENCE;
	return distance;
}
