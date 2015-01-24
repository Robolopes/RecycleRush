/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#include "InputManager.h"
#include "../PenguinLib/PenguinLib.h"
#include <fstream.h>

InputManager* InputManager::mSingleton = NULL;

InputManager::InputManager()
{
	
	mDriverInput = new DriverInput();
	mCurrentInput = mDriverInput;
	mCounterOfFiles = 0;
}

void InputManager::Init()
{
		
	for(int i = 0; i < 2; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			mAnalogReadings[i][j] = 0.0f;
		}
	}
	
	
	
	
	taskSpawn("1717InputReader", 80, VX_FP_TASK, 50000,
				(FUNCPTR)&InputManager::readTask, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
}

void InputManager::SetInputMode(InputMode mode)
{
	switch(mode)
	{
		case InputMode_Driver:
			mCurrentInput = mDriverInput;
			break;
		
	}
}

PolarCoordinate InputManager::GetDrive()
{
	return mCurrentInput->GetDrive();
}

float InputManager::GetLeftYAxis()
{
	return mCurrentInput->GetLeftYAxis();
}

float InputManager::GetLeftXAxis()
{
	return mCurrentInput->GetLeftXAxis();
}

float InputManager::GetRightYAxis()
{
	return mCurrentInput->GetRightYAxis();
}

float InputManager::GetRightXAxis()
{
	return mCurrentInput->GetRightXAxis();
}

float InputManager::GetCoRightXAxis()
{
	return mCurrentInput->GetCoLeftXAxis();
}

float InputManager::GetCoRightYAxis()
{
	return mCurrentInput->GetCoLeftYAxis();
}

float InputManager::GetCoLeftXAxis()
{
	return mCurrentInput->GetCoRightXAxis();
}

float InputManager::GetCoLeftYAxis()
{
	return mCurrentInput->GetCoRightYAxis();
}

InputManager* InputManager::GetSingletonPtr()
{
	if (mSingleton == NULL)
		mSingleton = new InputManager();
	
	return mSingleton;
}

bool InputManager::GetChangedToDown(UINT32 BtnInfo)
{
	return mCurrentInput->GetChangedToDown(BtnInfo);
}

bool InputManager::GetChangedToUp(UINT32 BtnInfo)
{
	return mCurrentInput->GetChangedToUp(BtnInfo);
}

bool InputManager::GetButton(UINT32 BtnInfo)
{
	return mCurrentInput->GetButton(BtnInfo);
}

float InputManager::GetScaledReading(UINT32 info)
{
	//convert from 1-8 to 0-7 (for array indexing etc)
	return GetScaledReading(UNBUNDLE_INFO(info));
}

float InputManager::GetScaledReading(UINT32 slot, UINT32 channel)
{
	float ratio = 5.0 / mAnalogReadings[1][6];
	//						convert from 1-8 to 0-7 (for array indexing etc)
	float scaledVoltage = mAnalogReadings[slot-1][channel-1] * ratio;
	return ClipSensorVoltage(scaledVoltage);
}

float InputManager::ReadReferenceVoltage()
{
	return mAnalogReadings[1][6];
}

void InputManager::readTask()
{
	InputManager* inputManager = gInputManager;
	AnalogChannel* analogChannels[2][8];
	
	//initialize analog channels
	for(int i = 0; i < 2; i++)
	{
		for(int j = 0; j < 8; j++)
		{
			analogChannels[i][j] = new AnalogChannel(i+1, j+1);
			analogChannels[i][j]->SetAverageBits(4);
		}
	}
	
	//repeatedly synchronously read
	FOREVER
	{
		for(int i = 0; i < 2; i++)
		{
			for (int j = 0; j < 8; j++)
			{
				inputManager->mAnalogReadings[i][j] = analogChannels[i][j]->GetAverageVoltage();
			}
		}
		
		PenguinLib::Sleep(10);
	}
}

float InputManager::ClipSensorVoltage(float val)
{
	if(val < 0)
		return 0.f;
	if(val > 5)
		return 5.f;
	return val;
}


float InputManager::GetUnscaledReading(UINT32 info)
{
	return GetUnscaledReading(UNBUNDLE_INFO(info));
}
float InputManager::GetUnscaledReading(UINT32 slot, UINT32 channel)
{
	return mAnalogReadings[slot-1][channel-1];
}

float InputManager::GetXAxis(UINT32 joy)
{
	switch(joy)
	{
		case 1:
			return GetLeftXAxis();
			break;
		case 2:
			return GetRightXAxis();
			break;
		case 3:
			return GetCoRightXAxis();
			break;
		default: //four
			return GetCoLeftXAxis();
			break;
	}
}

float InputManager::GetYAxis(UINT32 joy)
{
	switch(joy)
	{
		case 1:
			return GetLeftYAxis();
			break;
		case 2:
			return GetRightYAxis();
			break;
		case 3:
			return GetCoRightYAxis();
			break;
		default: //four
			return GetCoLeftYAxis();
			break;
	}
}


float InputManager::GetBatteryVoltage()
{
	return gDriverStation->GetBatteryVoltage();
}
