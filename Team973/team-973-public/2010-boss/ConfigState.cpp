/**
 *	@file ConfigState.cpp
 *	Implementation of the ConfigState class.
 *
 *	Team 973<br>
 *	2010 "The Boss"
 *
 *	Created on 2/26/10.
 */

#include "ConfigState.hpp"
#include "NormalState.hpp"
#include "ControlBoard.hpp"
#include "DriveSystem.hpp"
#include "KickerSystem.hpp"

ConfigState::ConfigState(BossRobot *r)
	: State(r)
{
}

void ConfigState::Enter()
{
#ifdef FEATURE_LCD
	DS_LCD *lcd = DS_LCD::GetInstance();
	lcd->Clear();
	lcd->PrintfLine(DS_LCD::kUser_Line1, "CONFIGURATION");
	lcd->UpdateLCD();
#endif
	
	m_robot->SoftStop();
	
	// Set up state
	m_reread = Flag();
	m_strengthLo = m_strengthMd = m_strengthHi = Flag();
	m_kickRest = Flag();
	m_robot->GetConfig().Read("boss.cfg");
}

void ConfigState::Exit()
{
	m_robot->GetConfig().Write("boss.cfg", "This is the main configuration file and was automatically generated.");
	m_robot->GetKickerEncoder()->ResetAccumulator();
	m_robot->GetKickerSystem()->Reset();
}

void ConfigState::Step()
{
	ControlBoard &board = ControlBoard::GetInstance();
	
	// Check for state stopping
	if (!board.GetButton(1))
	{
		m_robot->ChangeState(new NormalState(m_robot));
		return;
	}
	
	// Check for the re-read config file button
	m_reread.Set(board.GetButton(5));
	if (m_reread.GetTriggeredOn())
	{
		m_robot->GetConfig().Read("boss.cfg");
	}
	
	m_robot->GetShoulderBrake()->Set(1); // to unbrake
	
	HandleStrengthPresetting();
	HandleKickPresetting();
	HandleShoulderPresetting();
	HandleElbowPresetting();

#ifdef FEATURE_LCD
	DS_LCD::GetInstance()->PrintfLine(DS_LCD::kUser_Line6,
		"Auto: %.2x", m_robot->GetAutoValueA() << 4 | m_robot->GetAutoValueB());
	DS_LCD::GetInstance()->UpdateLCD();
#endif
} // END STEP

#ifdef FEATURE_UPPER_BOARD
#define STRENGTH_PRESET(button, name) \
	{ \
		m_strength##name.Set(ControlBoard::GetInstance().GetButton(button)); \
		if (m_strength##name.GetTriggeredOn()) \
			m_robot->GetConfig().Set("kickerStrength" #name "_pos", m_robot->GetKickerWinchSensor()->GetVoltage()); \
	}
#else
#define STRENGTH_PRESET(button, name)
#endif

void ConfigState::HandleStrengthPresetting()
{
	STRENGTH_PRESET(15, Lo);
	STRENGTH_PRESET(11, Md);
	STRENGTH_PRESET(7,  Hi);
	
#ifdef FEATURE_UPPER_BOARD
	if (ControlBoard::GetInstance().GetJoystick(3).GetRawButton(6))
	{
		m_robot->GetKickerWinch1()->Set(Relay::kForward);
		m_robot->GetKickerWinch2()->Set(Relay::kForward);
	}
	else if (ControlBoard::GetInstance().GetJoystick(3).GetRawButton(7))
	{
		m_robot->GetKickerWinch1()->Set(Relay::kReverse);
		m_robot->GetKickerWinch2()->Set(Relay::kReverse);
	}
	else
	{
		m_robot->GetKickerWinch1()->Set(Relay::kOff);
		m_robot->GetKickerWinch2()->Set(Relay::kOff);
	}
#endif

#ifdef FEATURE_LCD
	DS_LCD::GetInstance()->PrintfLine(DS_LCD::kUser_Line3,
		"Winch: %.2fV", m_robot->GetKickerWinchSensor()->GetVoltage());
#endif
}

void ConfigState::HandleKickPresetting()
{
	ControlBoard &board = ControlBoard::GetInstance();
	// Run kicker motor when holding trigger
	m_robot->GetKickerMotor()->Set(board.GetJoystick(3).GetTrigger() ? 1.0 : 0.0);
	
	// Set rest point for kicker
	m_kickRest.Set(board.GetJoystick(3).GetRawButton(2));
	if (m_kickRest.GetTriggeredOn())
	{
#ifdef FEATURE_UPPER_BOARD
		m_robot->GetConfig().Set("kickerRestAngle", m_robot->GetKickerEncoder()->GetVoltage());
#endif
	}
	
	// Set cocked point for kicker
	m_kickCocked.Set(board.GetJoystick(3).GetRawButton(3));
	if (m_kickCocked.GetTriggeredOn())
	{
#ifdef FEATURE_UPPER_BOARD
		m_robot->GetConfig().Set("kickerCockedAngle", m_robot->GetKickerEncoder()->GetVoltage());
#endif
	}
	
#ifdef FEATURE_LCD
	DS_LCD::GetInstance()->PrintfLine(DS_LCD::kUser_Line2,
		"Kicker: %.2fV", m_robot->GetKickerEncoder()->GetVoltage());
#endif
}

void ConfigState::HandleShoulderPresetting()
{
	ControlBoard &board = ControlBoard::GetInstance();
	
	m_robot->GetShoulderMotor1()->Set(board.GetJoystick(1).GetY());
	m_robot->GetShoulderMotor2()->Set(board.GetJoystick(1).GetY());
	
	m_shoulderStowed.Set(board.GetJoystick(1).GetRawButton(2));
	if (m_shoulderStowed.GetTriggeredOn())
	{
		m_robot->GetConfig().Set("shoulderStowedPos", m_robot->GetShoulderSensor()->GetVoltage());
	}
	
	m_shoulderRaised.Set(board.GetJoystick(1).GetRawButton(3));
	if (m_shoulderRaised.GetTriggeredOn())
	{
		m_robot->GetConfig().Set("shoulderRaisedPos", m_robot->GetShoulderSensor()->GetVoltage());
	}
	
	m_shoulderGTFU.Set(board.GetJoystick(1).GetTrigger());
	if (m_shoulderGTFU.GetTriggeredOn())
	{
		m_robot->GetConfig().Set("shoulderGTFUPos", m_robot->GetShoulderSensor()->GetVoltage());
	}
	
#ifdef FEATURE_LCD
	DS_LCD::GetInstance()->PrintfLine(DS_LCD::kUser_Line4,
		"Shoulder: %.2fV", m_robot->GetShoulderSensor()->GetVoltage());
#endif
}

void ConfigState::HandleElbowPresetting()
{
	ControlBoard &board = ControlBoard::GetInstance();
	double voltage = 5.0 - m_robot->GetElbowSensor()->GetVoltage();
	
	m_elbowTarget.Set(board.GetJoystick(2).GetTrigger());
	if (m_elbowTarget.GetTriggeredOn())
	{
		m_robot->GetConfig().Set("elbowTarget", voltage);
	}
	
	m_elbowMin.Set(board.GetJoystick(2).GetRawButton(4));
	if (m_elbowMin.GetTriggeredOn())
	{
		m_robot->GetConfig().Set("elbowMinimum", voltage);
	}
	
	m_elbowMax.Set(board.GetJoystick(2).GetRawButton(5));
	if (m_elbowMax.GetTriggeredOn())
	{
		m_robot->GetConfig().Set("elbowMaximum", voltage);
	}

#ifdef FEATURE_LCD
	DS_LCD::GetInstance()->PrintfLine(DS_LCD::kUser_Line5,
		"Elbow: %.2fV", voltage);
#endif
}
