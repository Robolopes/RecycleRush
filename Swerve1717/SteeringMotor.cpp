#include "SteeringMotor.h"
#include "System.h"
#include <cmath>
#include "PenguinLib/PenguinLib.h"
#include "Input/InputManager.h"

SteeringMotor::SteeringMotor(UINT32 jaguarSlotChannel, UINT32 potSlotChannel, float maxSpeed, bool wrapAround, float negLim, float posLim)
{
	mSpeedController = new Jaguar(PenguinLib::VirtualSlot(jaguarSlotChannel), PenguinLib::VirtualChannel(jaguarSlotChannel));

	mPotInfo = potSlotChannel;
	mLastValue = 2.5f;
	mLastTime = GetFPGATime();
	mLastVelocity = 0.0f;
	mNegLim = negLim;
	mPosLim = posLim;
	
	mLoopControl = new SimPID(PIDValues(), 1, wrapAround);
	mLoopControl->setMaxOutput(maxSpeed);
	
	PenguinLib::Sleep(60);
	mLastAngle = _getUnfilteredAngle();
}

void SteeringMotor::GoToAngle(float Angle)
{
	if(Angle < mNegLim)
		Angle = mNegLim;
	else if (Angle > mPosLim)
		Angle = mPosLim;
	
	mGoalAngle = Angle;
	Update();
}

void SteeringMotor::SetControlConstants(float voltsPerDegree,
							float centerVoltage, PIDValues controlConstants)
{
	mCenterVoltage = centerVoltage;
	mVoltsPerDegree = voltsPerDegree;
	mLoopControl->setConstants(controlConstants);
}
void SteeringMotor::SetSpeed(float speed)
{
	mSpeedController->Set(speed);
}

void SteeringMotor::Update()
{
	mLoopControl->setDesiredValue(mGoalAngle);
	mSpeedController->Set(mLoopControl->calcPID(GetCurrentAngle()));
	
	if (mLoopControl->isDone())
	{
		mLoopControl->resetErrorSum();
	}
}

float SteeringMotor::GetCurrentAngle()
{
	float voltage = gInputManager->GetScaledReading(mPotInfo);
	if (voltage < 0.0)
		voltage = 0.0;
	float voltageCentered = mCenterVoltage - voltage;
	float rval = voltageCentered / mVoltsPerDegree;
	return rval;
}

void SteeringMotor::GoToRelativeAngle(float angle)
{
	float CurAngle = GetCurrentAngle();
	GoToAngle(angle + CurAngle);
}

float SteeringMotor::GetDegreesPerSecond()
{
	float curPotValue = gInputManager->GetScaledReading(mPotInfo);
	float ChangeInAngle = (curPotValue - mLastValue) * 360.f / 5.f;
	float ChangeInTime = GetFPGATime() - mLastTime;
	
	mLastValue = curPotValue;
	mLastTime = GetFPGATime();
	mLastVelocity = ChangeInAngle * 1000000.f / ChangeInTime;
	
	return ChangeInAngle * 1000000.f / ChangeInTime;
}

void SteeringMotor::PrintInfo()
{
	//printf("%f\t%f\r\n", mSpeedController->Get(), mAnalogChannel->GetAverageVoltage());
}

float SteeringMotor::_getUnfilteredAngle()
{
	float voltage = gInputManager->GetScaledReading(mPotInfo);
	if (voltage < 0.0)
		voltage = 0.0;
	return (mCenterVoltage - voltage) / mVoltsPerDegree;
}

void SteeringMotor::GoToAngleWithoutUpdating(float angle)
{
	if(angle < mNegLim)
		angle = mNegLim;
	else if (angle > mPosLim)
		angle = mPosLim;
	
	mGoalAngle = angle;
	//Update();
}
