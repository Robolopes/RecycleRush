/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/


const float kCommanderAngle = 69;

#include "DrivetrainManager.h"

#include "../Input/InputManager.h"
#include "../Input/PenguinJoystick.h"
#include "../PenguinLib/PenguinLib.h"
#include <cmath>



#define NORMAL_BATTERY_VOLTAGE_old 11.8
#define NORMAL_BATTERY_VOLTAGE 12
#define MAX_JOYSTICK_INPUT .4
#define MAX_TORQUE .9
#define RPM_PER_TORQUE -2000/MAX_TORQUE
#define RPM_MAX 5000


DriveTrainManager::DriveTrainManager()
{
	mLeftDrive  = new DriveSide(PWM_LEFT_DRIVE, ENCODER_LEFT_A, ANALOG_LEFT_ANGLE,
				  PWM_LEFT_STEER, LEFT_STEER_CENTER, LEFT_STEER_SIGN * 5.00f/360, LEFT_STEER_PID);
	mRightDrive = new DriveSide(PWM_RIGHT_DRIVE, ENCODER_RIGHT_A, ANALOG_RIGHT_ANGLE,
				  PWM_RIGHT_STEER, RIGHT_STEER_CENTER, RIGHT_STEER_SIGN * 5.00f/360, RIGHT_STEER_PID);
	
	
	mPushingMode = PUSH_NONE;
}

void DriveTrainManager::_setLeftAngle(float angle)
{
	mLeftDrive->TurnTo(angle);
}

void DriveTrainManager::_setRightAngle(float angle)
{
	mRightDrive->TurnTo(angle);
}

void DriveTrainManager::_processVectorMode()
{
	_RawMode();
	
}

void DriveTrainManager::_RawMode()
{
	PolarCoordinate driveInput;
	driveInput = gInputManager->GetDrive();
	float skidInput = -gInputManager->GetRightXAxis();
	skidInput = PenguinMath::fsignsquare(skidInput);
	_setLeftAngle(driveInput.theta);
	_setRightAngle(driveInput.theta);
	_setLeftSpeed(driveInput.r - skidInput);
	_setRightSpeed(driveInput.r + skidInput);
}



void DriveTrainManager::ProcessTeleop()
{
	//PrintSpeed();
	static UINT32 time = GetFPGATime();
	
	_processVectorMode();
	
	if(gInputManager->GetChangedToDown(RIGHT_JOY + 10))
		PrintCenterVoltages();
	
}



void DriveTrainManager::_setLeftSpeed(float speed)
{
	mLeftDrive->Set(speed);
}

void DriveTrainManager::_setRightSpeed(float speed)
{
	mRightDrive->Set(speed);
}

void DriveTrainManager::PrintCenterVoltages()
{
	//printf("%f\t %f \t\r\n", gInputManager->GetScaledReading(ANALOG_LEFT_ANGLE), gInputManager->GetScaledReading(ANALOG_LEFT_ANGLE));
}



