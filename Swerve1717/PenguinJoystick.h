/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#ifndef PENGUIN_JOYSTICK_H_
#define PENGUIN_JOYSTICK_H_

#include "WPILib.h"
#include <map>
#include "../PenguinLib/PenguinLib.h"

typedef std::map<UINT32, bool> ButtonMap;

/**
 * Joystick with some customization.
 */
class PenguinJoystick : public Joystick
{
	public:
		PenguinJoystick(UINT32 port) : Joystick(port){}
		
		/**
		 * This checks to see if a button has changed from up to down.
		 * @param button The button number.
		 * @return If it has changed from up to down.
		 */
		bool GetButtonChangedToDown(UINT32 button);
		
		/**
		 * This checks to see if a button has changed from down to up.
		 * @param button The button number.
		 * @return If it has changed from down to up.
		 */
		bool GetButtonChangedToUp(UINT32 button);
		
		/**
		 * Convert to polar coordinate.
		 * @return The joystick X and Y axese as a polar coordinate.
		 */
		PolarCoordinate GetPolar();
		
	private:
		ButtonMap mButtonMapDown;
		ButtonMap mButtonMapUp;
};

#endif
