/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#include "PenguinLib.h"
#include "../System.h"

float PenguinMath::fsignsquare(float input)
{
	return fabs(input) * input;
}

float PenguinMath::flimitmix(float input)
{
	if (input < -1)
		input = -1;
	if (input > 1)
		input = 1;
	return input;
}

void PenguinMath::KeepInRange(int& a, int& b, int minA, int maxA, int minB, int maxB)
{
	if(a < minA)
		a = minA;
	if(a > maxA)
		a = maxA;
	if(b < minB)
		b = minB;
	if(b > maxB)
		b = maxB;
}

float PenguinMath::Scale(float val, float min)
{
	return (min) + (1 - min) * (val);
}

PolarCoordinate PenguinMath::rectToPolar(float x, float y)
{
	float sign = (y < 0) ? -1 : 1;
	float theta;
	if (y == 0)
	{
		theta = (x < 0) ? -90 : 90;
	}
	else
	{
		if(sign == -1)
		{
			theta = atan(y / x) * 180 / PI;
			if(theta < 0)
				theta -= 90;
			else
				theta += 90;
			
			//mirror the angle to fix quadrants III and IV
			theta = -theta;
		}
		else
		{
			theta = atan(x / y) * 180 / PI;
		}
	}
	
	float r = sqrt(x*x + y*y);
	if(r > 1)
		r = 1;
	if(r < -1)
		r = -1;
	if(r < .05)
	{
		theta = 0;
		r = 0;
	}
	else
	{
		//r = (r - .05) / .95;
		//r *= r;
	}
	
	//theta = fmod(theta + 360, 360);  //uncomment this line to change -180->180 to 0->360
	return PolarCoordinate(theta, r);
}
