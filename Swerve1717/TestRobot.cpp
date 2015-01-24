/*
---------------------------------------------------------------------------------------
This source file is part of the DP Engineering Academy Penguin Bot!


Copyright (c) 2008 Dos Pueblos Engineering Academy

---------------------------------------------------------------------------------------
*/

#include "TestRobot.h"
#include "Input/InputManager.h"
#include "System.h"
#include <cmath>


TestRobot::TestRobot()
{
	GetWatchdog().SetExpiration(.1);
	
	gInputManager->Init();
	mDrivetrainManager = new DriveTrainManager();

}

void TestRobot::RobotInit(void)
{
	
	
}

void TestRobot::DisabledInit(void)
{
	
}

void TestRobot::AutonomousInit(void)
{
	GetWatchdog().SetEnabled(false);

}

void TestRobot::TeleopInit(void)
{
	
	gInputManager->SetInputMode(InputMode_Driver);
	GetWatchdog().SetEnabled(true);
}

void TestRobot::DisabledPeriodic(void)
{
}

void TestRobot::AutonomousPeriodic(void)
{
	TeleopPeriodic();
}

void TestRobot::TeleopPeriodic(void)
{
	GetWatchdog().Feed();
	mDrivetrainManager->ProcessTeleop();
	
}

 START_ROBOT_CLASS(TestRobot);
