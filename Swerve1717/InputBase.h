#ifndef INPUT_BASE_H_
#define INPUT_BASE_H_

#include "../Input/PenguinJoystick.h"

typedef struct JoystickState_struct{
	public:
		UINT16 mButtons;
		float xAxis;
		float yAxis;
} JoystickState;

#define NUM_JOYSTICKS 4
#define NUM_SAMPLES 150

typedef JoystickState JoystickRecording[NUM_SAMPLES][NUM_JOYSTICKS];

class InputBase
{
	public:
		virtual ~InputBase() {};
		/**
		 * @return Y axis, left joystick.
		 */
		virtual float GetLeftYAxis() = 0;
		
		/**
		 * @return X axis, left joystick.
		 */
		virtual float GetLeftXAxis() = 0;
		
		/**
		 * @return Y axis, right joystick.
		 */
		virtual float GetRightYAxis() = 0;
		
		
		/**
		 * @return X axis, right joystick.
		 */
		virtual float GetRightXAxis() = 0;
		
		/**
		 * Get the drive input.
		 * @return The input to the drivetrain as a polar coordinate.
		 */
		virtual PolarCoordinate GetDrive() = 0;
		
		/**
		 * @return the axis for manually controlling the turret.
		 */
		virtual float GetTurretAxis() = 0;
		
		virtual float GetCoLeftXAxis() = 0;
		virtual float GetCoLeftYAxis() = 0;
		virtual float GetCoRightXAxis() = 0;
		virtual float GetCoRightYAxis() = 0;
		
		/**
		 * Get the current state of a button.
		 * @param BtnInfo A bytemask for the joystick and button number.
		 */
		virtual bool GetButton(UINT32 BtnInfo) = 0;
		
		/**
		 * Get whether or not a button has just changed to up.
		 * @param BtnInfo A bytemask for the joystick and button number.
		 */
		virtual bool GetChangedToUp(UINT32 BtnInfo) = 0;
		
		/**
		 * Get whether or not a button has just changed to down.
		 * @param BtnInfo A bytemask for the joystick and button number.
		 */
		virtual bool GetChangedToDown(UINT32 BtnInfo) = 0;

};
#endif /* INPUT_BASE_H_ */
