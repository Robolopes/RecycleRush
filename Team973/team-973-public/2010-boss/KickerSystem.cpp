/**
 *	@file KickerSystem.cpp
 *	Implementation of the KickerSystem class.
 *
 *	Team 973<br>
 *	2010 "The Boss"
 *
 *	Created on 2/16/10.
 */

#include "KickerSystem.hpp"
#include "ControlBoard.hpp"
#include <math.h>

KickerSystem::KickerSystem(BossRobot *r)
{
	m_robot = r;
	m_intakeTimer = new Timer();
	m_intakeTimer->Start();
	m_intakePossess = false;
	Reset();
}

KickerSystem::~KickerSystem()
{
	delete m_intakeTimer;
}

void KickerSystem::Reset()
{
	ResetKicker();
	
	m_strength = kStrengthLo;
	m_intakeState = 0;
}

void KickerSystem::ResetKicker()
{
	double restAngle = m_robot->GetConfig().SetDefault("kickerRestAngle", 4.5);
	
	m_kicking = false;
	m_startedKicking = false;
	m_manualRunKicker = false;
	m_cocking = false;
	m_cockingBegan = false;
	m_cockingEnded = false;

	m_kickerPID.SetPID(m_robot->GetConfig().SetDefault("kickerP", 0.8),
					   m_robot->GetConfig().SetDefault("kickerI", 0.0),
					   m_robot->GetConfig().SetDefault("kickerD", 0.0));
	m_kickerPID.SetLimits(0.0, 1.0);
	
	m_robot->GetKickerEncoder()->ResetAccumulator();
	m_kickerPID.SetTarget((m_robot->GetKickerEncoder()->GetIncrementalVoltage() >= restAngle)
						  ? restAngle
						  : restAngle - m_robot->GetKickerEncoder()->GetMaxVoltage());
}

void KickerSystem::RunIntake()
{
	m_intakeState = 1;
}

void KickerSystem::StopIntake()
{
	m_intakeState = 0;
}

bool KickerSystem::HasPossession()
{
	return m_intakePossess;
}

void KickerSystem::Cock()
{
	m_cocking = true;
}
	
void KickerSystem::ReadControls()
{
	ControlBoard &board = ControlBoard::GetInstance();
	short hudState = ControlBoard::kLightOff;
	
	// Update winch controls
	if (board.GetButton(15))
		SetStrength(kStrengthLo);
	else if (board.GetButton(11))
		SetStrength(kStrengthMd);
	else if (board.GetButton(7))
		SetStrength(kStrengthHi);
	
	m_resetFlag.Set(board.GetJoystick(3).GetRawButton(10));
	if (m_resetFlag.GetTriggeredOn())
		ResetKicker();
	
	board.SetMultiLight(6, ControlBoard::kLightOff);
	board.SetMultiLight(14, ControlBoard::kLightOff);
	board.SetMultiLight(10, ControlBoard::kLightOff);
	
	switch (m_strength)
	{
	case kStrengthLo:
		board.SetMultiLight(6, ControlBoard::kLightGreen);
		break;
	case kStrengthMd:
		board.SetMultiLight(14, ControlBoard::kLightGreen);
		break;
	case kStrengthHi:
		board.SetMultiLight(10, ControlBoard::kLightGreen);
		break;
	}
	
	// Update kicker controls
	m_kickTrigger.Set(board.GetJoystick(3).GetTrigger());
	if (m_kickTrigger.GetTriggeredOn())
		Kick();
	
	// Update manual kicker mode
	m_manualMode.Set(board.GetJoystick(3).GetRawButton(6) && board.GetJoystick(3).GetRawButton(7));
	if (m_manualMode.Get())
	{
		m_manualRunKicker = m_kickTrigger.Get();
	}
	else
	{
		m_manualRunKicker = false;
	}
	
	// Update intake controls
	if (board.GetJoystick(3).GetRawButton(2) ||
		board.GetJoystick(3).GetRawButton(3) ||
		board.GetJoystick(3).GetRawButton(4) ||
		board.GetJoystick(3).GetRawButton(5))
	{
		RunIntake();
	}
	else if (board.GetJoystick(3).GetRawButton(8))
	{
		m_intakeState = -1;
	}
	else
	{
		StopIntake();
	}

	m_intakeFlag.Set(m_intakeState != 0);
	if (m_intakeFlag.GetTriggeredOn())
		Cock();
	
	// Change display on the HUD
	hudState = HasPossession() ? ControlBoard::kLightGreen : ControlBoard::kLightRed;
	board.SetMultiLight(16, 12, hudState);
	
#ifdef FEATURE_LCD
	DS_LCD *lcd = DS_LCD::GetInstance();
	lcd->PrintfLine(DS_LCD::kUser_Line5, "Kicker: %.2fV", m_robot->GetKickerEncoder()->GetVoltage());
	lcd->UpdateLCD();
#endif
}

void KickerSystem::Kick()
{
	if (!m_kicking)
	{
		m_kicking = true;
		m_robot->GetKickerEncoder()->ResetAccumulator();
	}
}

void KickerSystem::SetStrength(short s)
{
	m_strength = s;
}

void KickerSystem::Update()
{
	UpdateIntake();
	UpdateWinch();
	UpdateKicker();
	UpdatePossession();
}

bool KickerSystem::NeedsWinchUpdate()
{
	double actual = m_robot->GetKickerWinchSensor()->GetVoltage();
	double tolerance = m_robot->GetConfig().SetDefault("winchPosTolerance", 0.02);
	double target = GetWinchTarget();
	
	return (fabs(actual - target) > tolerance * 2);
}

double KickerSystem::GetWinchTarget()
{
	switch (m_strength)
	{
	case kStrengthLo:
		return m_robot->GetConfig().SetDefault("kickerStrengthLo_pos", 0.0);
	case kStrengthMd:
		return m_robot->GetConfig().SetDefault("kickerStrengthMd_pos", 0.0);
	case kStrengthHi:
		return m_robot->GetConfig().SetDefault("kickerStrengthHi_pos", 0.0);
	default:
		return -1.0;
	}
}

void KickerSystem::UpdateWinch()
{
	double target, actual, tolerance;
	
#ifndef FEATURE_UPPER_BOARD
	return;
#endif
	
	if (m_startedKicking)
	{
		m_robot->GetKickerWinch1()->Set(Relay::kOff);
		m_robot->GetKickerWinch2()->Set(Relay::kOff);
		return;
	}
	
	if (m_strength != kStrengthLo && m_strength != kStrengthMd && m_strength != kStrengthHi)
	{
		// We entered a weird strength. Don't do anything, plz!
		m_robot->GetKickerWinch1()->Set(Relay::kOff);
		m_robot->GetKickerWinch2()->Set(Relay::kOff);
		return;
	}

	target = GetWinchTarget();
	actual = m_robot->GetKickerWinchSensor()->GetVoltage();
	tolerance = m_robot->GetConfig().SetDefault("winchPosTolerance", 0.02);
	if (actual < (target - tolerance))
	{
		m_robot->GetKickerWinch1()->Set(Relay::kForward);
		m_robot->GetKickerWinch2()->Set(Relay::kForward);
	}
	else if (actual > (target + tolerance))
	{
		m_robot->GetKickerWinch1()->Set(Relay::kReverse);
		m_robot->GetKickerWinch2()->Set(Relay::kReverse);
	}
	else
	{
		m_robot->GetKickerWinch1()->Set(Relay::kOff);
		m_robot->GetKickerWinch2()->Set(Relay::kOff);
	}
}

void KickerSystem::UpdateKicker()
{
	AbsoluteEncoder *encoder = m_robot->GetKickerEncoder();
	float encoderVoltage, encoderMaxVoltage;
	double restVoltage, cockedVoltage, tol;
	
	if (m_manualMode.Get())
	{
		// ADAM FORCED ME TO DO THIS.  I CUT MY WRISTS AFTER THIS LINE OF CODE.
		m_robot->GetKickerMotor()->Set(m_manualRunKicker ? 1.0 : 0.0);
		return;
	}
	else if (m_manualMode.CheckTriggeredOff())
	{
		ResetKicker();
	}
	
	encoderVoltage = encoder->GetIncrementalVoltage();
	encoderMaxVoltage = encoder->GetMaxVoltage();
	
	// Get config values
	restVoltage = m_robot->GetConfig().SetDefault("kickerRestAngle", 4.5);
	cockedVoltage = m_robot->GetConfig().SetDefault("kickerCockedAngle", 4.710);
	tol = m_robot->GetConfig().SetDefault("kickerPosTolerance", 0.05);

	m_kickerPID.SetPID(m_robot->GetConfig().GetDouble("kickerP"),
					   m_robot->GetConfig().GetDouble("kickerI"),
					   m_robot->GetConfig().GetDouble("kickerD"));
	
	// Determine target
	// Note that this relies on the fact that the cocked angle is farther than rest, so the motor won't
	// backpedal.
	if (m_kicking)
	{
		// This is our first kicking cycle.  Set the target.
		if (!m_startedKicking)
		{
			encoder->ResetAccumulator();
			encoderVoltage = encoder->GetIncrementalVoltage();
			if (encoderVoltage < restVoltage - tol)
			{
				m_kickerPID.SetTarget(restVoltage);
			}
			else
			{
				m_kickerPID.SetTarget(restVoltage + encoderMaxVoltage);
			}
		}
		m_startedKicking = true;
		
		// Check to see if we're finished kicking
		if (encoderVoltage > m_kickerPID.GetTarget() - tol)
		{
			// We've finished kicking. Clean up.
			ResetKicker();
		}
	}
	else if (m_cocking && !m_cockingBegan)
	{
		// Move the kicker to cocked if the operator starts the intake
		encoder->ResetAccumulator();
		encoderVoltage = encoder->GetIncrementalVoltage();
		m_kickerPID.SetTarget((cockedVoltage >= restVoltage)
							  ? cockedVoltage
							  : cockedVoltage + encoderMaxVoltage);
		m_cockingBegan = true;
	}
	else if (m_cockingBegan && encoderVoltage > m_kickerPID.GetTarget() - tol)
	{
		// Done cocking, hold position.
		encoder->ResetAccumulator();
		encoderVoltage = encoder->GetIncrementalVoltage();
		m_kickerPID.SetTarget((encoderVoltage >= cockedVoltage)
							  ? cockedVoltage
							  : cockedVoltage - encoderMaxVoltage);
		m_cockingEnded = true;
	}
	
	// Run the motor!
	m_robot->GetKickerMotor()->Set(m_kickerPID.Update(encoderVoltage));
}

void KickerSystem::UpdateIntake()
{
	switch (m_intakeState)
	{
	case -1:
		m_robot->GetIntakeMotor()->Set(-1.0);
		break;
	case 0:
		m_robot->GetIntakeMotor()->Set(0.0);
		break;
	case 1:
		m_robot->GetIntakeMotor()->Set(1.0);
		break;
	default:
		m_robot->GetIntakeMotor()->Set(0.0);
		break;
	}
}

void KickerSystem::UpdatePossession()
{
	double timeElapsed, rate;
	
	if (m_intakeState != 1)
	{
		m_intakePossess = false;
		m_intakeTimer->Reset();
		m_intakeTimer->Stop();
		return;
	}
	
	m_intakeTimer->Start();
	timeElapsed = m_intakeTimer->Get();
	if (timeElapsed <= 0)
		timeElapsed = 0.002;
	
	if (timeElapsed > 0.1)
	{
		rate = m_robot->GetIntakeEncoder()->GetDistance() / timeElapsed;
		m_intakePossess = (m_intakeState == 1) && (rate < 180);
		DS_LCD::GetInstance()->PrintfLine(DS_LCD::kUser_Line6, "Intake: %.2f", rate);
		
		// Reset state
		m_intakeTimer->Reset();
		m_robot->GetIntakeEncoder()->Reset();
	}
}
