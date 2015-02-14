/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __MY_DRIVER_STATION_LCD_H__
#define __MY_DRIVER_STATION_LCD_H__

#include "SensorBase.h"

/**
 * Provide access to LCD on the Driver Station.
 * 
 * Buffer the printed data locally and then send it
 * when UpdateLCD is called.
 */
class MyDriverStationLCD : public SensorBase
{
public:
	static const UINT32 kSyncTimeout_ms = 20;
	static const UINT16 kFullDisplayTextCommand = 0x9FFF;
	static const INT32 kLineLength;
	static const INT32 kNumLines = 6;
	enum Line {kMain_Line6=0, kUser_Line1=0, kUser_Line2=1, kUser_Line3=2, kUser_Line4=3, kUser_Line5=4, kUser_Line6=5};

	virtual ~MyDriverStationLCD();
	static MyDriverStationLCD *GetInstance();

	void UpdateLCD();
	void Printf(Line line, INT32 startingColumn, const char *writeFmt, ...);
	void PrintfLine(Line line, const char *writeFmt, ...);
 
	void Clear();


protected:
	MyDriverStationLCD();

private:
	static void InitTask(MyDriverStationLCD *ds);
	static MyDriverStationLCD *m_instance;
	DISALLOW_COPY_AND_ASSIGN(MyDriverStationLCD);

	char *m_textBuffer;
	SEM_ID m_textBufferSemaphore;
};

#endif

