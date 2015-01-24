/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#ifndef PENGUIN_MATH_H_
#define PENGUIN_MATH_H_

#include "WPILib.h"
#include "math.h"
#include "syslib.h"

#define UNBUNDLE_INFO(x) PenguinLib::VirtualSlot(x),PenguinLib::VirtualChannel(x)
#define MAKE_BUNDLE( x, y ) ((x) << 16) + (y)

typedef struct PolarCoord_struct
{
	PolarCoord_struct(float theta, float r) { this->theta = theta; this->r = r; }
	PolarCoord_struct() { theta = 0.0, r = 0.0; }
	float theta;
	float r;
} PolarCoordinate;

/**
 * Some of our own math functions.
 */
namespace PenguinMath
{
	/**
	 * Square a float while maintaing its sign.
	 * @param input The number to squre.
	 * @return input squared with original sign.
	 */
	float fsignsquare(float input);
	
	/**
	 * Limit a float to -1 to 1.
	 * @param input The input number.
	 * @return The input number capped to -1 to 1.
	 */
	float flimitmix(float input);
	
	/**
	 * Keep two numbers in two ranges.
	 * @param a The first number.
	 * @param minA The minimum value for A.
	 * @param maxA The maximum value for A.
	 * @param b The second number.
	 * @param minB The minimum value for B.
	 * @param maxB The maximum value for A.
	 */
	void KeepInRange(int& a, int& b, int minA, int maxA, int minB, int maxB);
	
	/**
	 * Scale from zero-to-one to minimum-to-one.
	 * @param val The value to scale.
	 * @param min The new minimum for the value.
	 * @return the scaled value.
	 */
	float Scale(float val, float min);
	
	/**
	 * Convert rectangular coordinate to polar.
	 * @param x The x coordinate of the rectangular coordinate.
	 * @param y The y coordinate of the rectangular coordinate.
	 * @return The polar coordinate.
	 */
	PolarCoordinate rectToPolar(float x, float y);
}

/**
 * Some random stuff.
 */
namespace PenguinLib
{
	/**
	 * Blocks the current thread for a certain amount of time.
	 * @param ms Time in milliseconds.
	 */
	void Sleep(UINT64 ms);
	
	/**
	 * Process our bytemask, 0xFFFF0000 is slot, 0x0000FFFF is channel.
	 * @param slotChannel The bytemask to process.
	 * @return the upper four bytes.
	 */
	UINT32 VirtualSlot(UINT32 slotChannel);
	
	/**
	 * Process our bytemask, 0xFFFF0000 is slot, 0x0000FFFF is channel.
	 * @param slotChannel The bytemask to process.
	 * @return the lower four bytes.
	 */
	UINT32 VirtualChannel(UINT32 slotChannel);

}


#endif
