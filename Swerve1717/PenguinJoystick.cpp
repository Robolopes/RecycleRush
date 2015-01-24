/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#include "PenguinJoystick.h"
#include <cmath>

#include "../PenguinLib/PenguinLib.h"

bool PenguinJoystick::GetButtonChangedToDown(UINT32 button)
{
	bool current = GetRawButton(button);
	bool last = mButtonMapDown[button];
	mButtonMapDown[button] = current;
	
	if ((last == false) && (current == true))
		return true;
	else
		return false;
}

bool PenguinJoystick::GetButtonChangedToUp(UINT32 button)
{
	bool current = GetRawButton(button);
	bool last = mButtonMapUp[button];
	mButtonMapUp[button] = current;
	
		if ((last == true) && (current == false))
			return true;
		else
			return false;
}

PolarCoordinate PenguinJoystick::GetPolar()
{
	float x = GetX();
	float y = -GetY();
	PolarCoordinate p = PenguinMath::rectToPolar(x, y);
	if (fabs(p.theta) <= 1.5)
	{
		p.theta = 0.0;
	}
	p.theta = p.theta * 357.0 / 360.0;
	//this adds a three degree deadzone to the top of the joystick
	//and scales the rest so that there's not a jump at the edge.
	
	return p;
}
