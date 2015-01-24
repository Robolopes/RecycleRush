#include "DriverInput.h"
#include "../PenguinLib/PenguinLib.h"


DriverInput::DriverInput()
{
	mLeftJoy = new PenguinJoystick(LEFT_JOY >> 16);
	mRightJoy = new PenguinJoystick(RIGHT_JOY >> 16);
	mCoLeftJoy = new PenguinJoystick(CO_RIGHT_JOY >> 16);
	mCoRightJoy = new PenguinJoystick(CO_LEFT_JOY >> 16);
	
}

float DriverInput::GetLeftYAxis()
{
	return mLeftJoy->GetY();
}

float DriverInput::GetLeftXAxis()
{
	return mLeftJoy->GetX();
}

float DriverInput::GetRightYAxis()
{
	return mRightJoy->GetY();
}

float DriverInput::GetRightXAxis()
{
	return mRightJoy->GetX();
}

bool DriverInput::GetChangedToDown(UINT32 BtnInfo)
{
	UINT32 btn = PenguinLib::VirtualChannel(BtnInfo);
	bool rval = false;
	switch(PenguinLib::VirtualSlot(BtnInfo))
	{
		case 1:
			rval = mLeftJoy->GetButtonChangedToDown(btn);
			break;
		case 2:
			rval = mRightJoy->GetButtonChangedToDown(btn);
			break;
		case 3:
			rval = mCoLeftJoy->GetButtonChangedToDown(btn);
			break;
		case 4:
			rval = mCoRightJoy->GetButtonChangedToDown(btn);
			break;
		case 5:
			//rval = gDriverStation->GetDIChangedToDown(btn);
			break;
	}
	return rval;
}

PolarCoordinate DriverInput::GetDrive()
{
	return mLeftJoy->GetPolar();
}

float DriverInput::GetCoLeftXAxis()
{
	return mCoLeftJoy->GetX();
}

float DriverInput::GetCoLeftYAxis()
{
	return mCoLeftJoy->GetY();
}

float DriverInput::GetCoRightXAxis()
{
	return mCoRightJoy->GetX();
}

float DriverInput::GetCoRightYAxis()
{
	return mCoRightJoy->GetY();
}
	
bool DriverInput::GetChangedToUp(UINT32 BtnInfo)
{
	UINT32 btn = PenguinLib::VirtualChannel(BtnInfo);
	bool rval = false;
	switch(PenguinLib::VirtualSlot(BtnInfo))
	{
		case 1:
			rval = mLeftJoy->GetButtonChangedToUp(btn);
			break;
		case 2:
			rval = mRightJoy->GetButtonChangedToUp(btn);
			break;
		case 3:
			rval = mCoLeftJoy->GetButtonChangedToUp(btn);
			break;
		case 4:
			rval = mCoRightJoy->GetButtonChangedToUp(btn);
			break;
		case 5:
			//rval = gDriverStation->GetDIChangedToUp(btn);
			break;
	}
	return rval;
}

bool DriverInput::GetButton(UINT32 BtnInfo)
{
	UINT32 btn = PenguinLib::VirtualChannel(BtnInfo);
	switch(PenguinLib::VirtualSlot(BtnInfo))
	{
		case 1:
			return mLeftJoy->GetRawButton(btn);
		case 2:
			return mRightJoy->GetRawButton(btn);
		case 3:
			return mCoLeftJoy->GetRawButton(btn);
			break;
		case 4:
			return mCoRightJoy->GetRawButton(btn);
			break;
		case 5:
			return gDriverStation->GetDigitalIn(btn);
	}
	return (bool)90000;
}


/*UINT16 DriverInput::GetAllButtonsOnJoystick(UINT32 joystickNumber)
{
	switch(joystickNumber)
	{
		case 1:
			return mLeftJoy->GetAllButtons();
		case 2:
			return mRightJoy->GetAllButtons();
		case 3:
			return mCoLeftJoy->GetAllButtons();
			break;
		case 4:
			return mCoRightJoy->GetAllButtons();
			break;
		default:
			return 7; //this should never get here
			break;
			
	}
	
}*/
