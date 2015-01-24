/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#ifndef INPUTMANAGER_H_
#define INPUTMANAGER_H_

#include "WPILib.h"
#include "../Input/PenguinJoystick.h"
#include "../sim_pid/SimPID.h"
#include "InputBase.h"
#include "DriverInput.h"
#include <map>
#include <semLib.h>

#define gInputManager InputManager::GetSingletonPtr()

typedef enum
{
	InputMode_Driver,

} InputMode;



/**
 * Manage all of the input from the joysticks and Driver Station.
 */
class InputManager
{
	public:
		void Init();
		/**
		 * Retrieve the singleton instance.
		 */
		static InputManager* GetSingletonPtr();
		
		void SetInputMode(InputMode mode);
		
		/**
		 * Return the synchronously read voltage of a analog channel.
		 * @param info The slot and channel information.
		 */
		float 	GetScaledReading(UINT32 info);
		
		/**
		 * Return the synchronously read voltage of a analog channel.
		 * @param slot The slot to read.
		 * @param channel The channel to read.
		 */
		float 	GetScaledReading(UINT32 slot, UINT32 channel);
		
		/**
		 * @return Y axis, left joystick.
		 */
		float	GetLeftYAxis();
		
		/**
		 * @return X axis, left joystick.
		 */
		float	GetLeftXAxis();
		
		/**
		 * @return Y axis, right joystick.
		 */
		float	GetRightYAxis();
		
		/**
		 * @return X axis, right joystick.
		 */
		float	GetRightXAxis();
		
		/**
		 * Get the drive input.
		 * @return The input to the drivetrain as a polar coordinate.
		 */
		PolarCoordinate GetDrive();
		
		
		float GetCoRightXAxis();
		float GetCoRightYAxis();
		float GetCoLeftXAxis();
		float GetCoLeftYAxis();
		
		/**
		 * Get the current state of a button.
		 * @param BtnInfo A bytemask for the joystick and button number.
		 */
		bool GetButton(UINT32 BtnInfo);
		
		/**
		 * Get whether or not a button has just changed to up.
		 * @param BtnInfo A bytemask for the joystick and button number.
		 */
		bool GetChangedToUp(UINT32 BtnInfo);
		
		/**
		 * Get whether or not a button has just changed to down.
		 * @param BtnInfo A bytemask for the joystick and button number.
		 */
		bool GetChangedToDown(UINT32 BtnInfo);
				
		float ReadReferenceVoltage();
		float GetUnscaledReading(UINT32 info);
		float GetUnscaledReading(UINT32 slot, UINT32 channel);
		
		float GetXAxis(UINT32 joystick);
		float GetYAxis(UINT32 joystick);

		
		/**
		 * Return the current battery voltage.  This method will return a
		 * value extremely similar to that of the DriverStation's GetBatteryVoltage
		 * method, but without the semaphore business, which causes issues.
		 * @return The current battery voltage.
		 */
		float GetBatteryVoltage();
		
		UINT32 GetMultiSwitchPosition();
		
		void SetAutonomousChoice(UINT32 choice);
		
	private:
		DriverInput*	mDriverInput;
		InputBase*		mCurrentInput;
		
		
		
		InputManager();
		virtual ~InputManager(){}
		float ClipSensorVoltage(float val); 
		
		
		static InputManager* mSingleton;
		
		static void readTask();
		
		
		float			mAnalogReadings[2][8];
		int				mCounter;
		int				mCounterOfFiles;

};

#endif
