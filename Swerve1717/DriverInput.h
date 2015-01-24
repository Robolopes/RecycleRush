#ifndef DRIVER_INPUT_H_
#define DRIVER_INPUT_H_

#include "InputBase.h"

class DriverInput : public InputBase
{
	public:
		DriverInput();
		/**
		 * @return Y axis, left joystick.
		 */
		float GetLeftYAxis();
		
		/**
		 * @return X axis, left joystick.
		 */
		float GetLeftXAxis();
		
		/**
		 * @return Y axis, right joystick.
		 */
		float GetRightYAxis();
		
		
		/**
		 * @return X axis, right joystick.
		 */
		float GetRightXAxis();
		
		/**
		 * Get the drive input.
		 * @return The input to the drivetrain as a polar coordinate.
		 */
		PolarCoordinate GetDrive();
		
		/**
		 * @return the axis for manually controlling the turret.
		 */
		float GetTurretAxis() { return GetCoRightXAxis(); }
		
		float GetCoLeftXAxis();
		float GetCoLeftYAxis();
		float GetCoRightXAxis();
		float GetCoRightYAxis();
		

		//UINT16 GetAllButtonsOnJoystick(UINT32 joystickNumber);
		
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
		
	private:
		/**
		 * Left joystick, port 1.
		 */
		PenguinJoystick* mLeftJoy;
		
		/**
		 * Right joystick, port 2.
		 */
		PenguinJoystick* mRightJoy;
		
		/**
		 * Co-drive joystick, port 3.
		 */
		PenguinJoystick* mCoLeftJoy;
		PenguinJoystick* mCoRightJoy;
		
		PolarCoordinate	mLastDriveInput;
};
#endif /* DRIVER_INPUT_H_ */
