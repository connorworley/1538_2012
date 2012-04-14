//=============================================================================
// File: OperatorController.cpp
//
// COPYRIGHT 2012 Robotics Alliance of the West Coast(RAWC)
// All rights reserved.  RAWC proprietary and confidential.
//             
// The party receiving this software directly from RAWC (the "Recipient")
// may use this software and make copies thereof as reasonably necessary solely
// for the purposes set forth in the agreement between the Recipient and
// RAWC(the "Agreement").  The software may be used in source code form
// solely by the Recipient's employees/volunteers.  The Recipient shall have 
// no right to sublicense, assign, transfer or otherwise provide the source
// code to any third party. Subject to the terms and conditions set forth in
// the Agreement, this software, in binary form only, may be distributed by
// the Recipient to its users. RAWC retains all ownership rights in and to
// the software.
//
// This notice shall supercede any other notices contained within the software.
//=============================================================================
#include "OperatorController.h"
#include "WPILib.h"
#include "../RAWCLib.h"
#include "../RAWCControlBoard.h"
#include "../RAWCRobot.h"
#include "../Declarations.h"
#include "../RAWCConstants.h"

// Constructor
// TODO: We might not need to pass in Joysticks, if they come from the ControlBoard
OperatorController::OperatorController()
{
	constants = RAWCConstants::getInstance();
	bot = RAWCRobot::getInstance();
	cb = RAWCControlBoard::getInstance();
	telescopeExtend = false;
	previousFunnel = false;
	funnelState = false;
	previousHood = false;
	previousRMP = false;
	previousShooter = false;
	RMPState = false;
	shooterSpeedOffset = 0;
	shooterSpeedOffsetFender = 0;
	shooterSpeedOffsetFenderFunnel = 0;
	previousInc = false;
	previousDec = false;
}

// call this when you want to update the bot from a driver
void OperatorController::handle()
{
	if(cb->getOperatorButton(9) && cb->getOperatorButton(10))
	{
//		constants->insertKeyAndValue("shooterKey", constants->getValueForKey("shooterKey") + shooterSpeedOffset);
//		constants->insertKeyAndValue("shooterFender", constants->getValueForKey("shooterFender") + shooterSpeedOffsetFender);
//		constants->insertKeyAndValue("shooterFenderArmDown", constants->getValueForKey("shooterFenderArmDown") + shooterSpeedOffsetFenderFunnel);
//
//		constants->save();
	} 
	else 
	{
		if(cb->getOperatorButton(9) && previousDec != cb->getOperatorButton(9))
		{
			if(!bot->getShooter()->GetHoodState())
			{
				shooterSpeedOffset -= 200;
			} 
			else 
			{
				if(!funnelState)
				{
					shooterSpeedOffsetFender -= 200;
				} 
				else 
				{
					shooterSpeedOffsetFenderFunnel -= 200;
				}
			}
		}
		previousDec = cb->getOperatorButton(9);
		
		if(cb->getOperatorButton(10) && previousInc != cb->getOperatorButton(10))
		{
			if(!bot->getShooter()->GetHoodState())
			{
				shooterSpeedOffset += 200;
			} 
			else 
			{
				if(!funnelState)
				{
					shooterSpeedOffsetFender += 200;
				} 
				else 
				{
					shooterSpeedOffsetFenderFunnel += 200;
				}
			}
		}
		previousInc = cb->getOperatorButton(10);
	}
	
	if(cb->getOperatorButton(1) && previousFunnel != cb->getOperatorButton(1))
	{
		funnelState = !funnelState;
		bot->getFunnel()->Set(funnelState);
	}
	previousFunnel = cb->getOperatorButton(1);
	
	if(cb->getOperatorButton(2) && previousHood != cb->getOperatorButton(2))
	{
		if(bot->getShooter()->GetHoodState())
		{
			bot->getShooter()->RetractHood();
		} 
		else 
		{
			bot->getShooter()->ExtendHood();
		}
	}
	previousHood = cb->getOperatorButton(2);
	
	//if(previousShooter != cb->getOperatorButton(4))
	//{
		if(cb->getOperatorButton(4))
		{
			if(!bot->getShooter()->GetHoodState())
			{
				bot->getShooter()->SetSpeed(static_cast<float>(constants->getValueForKey("shooterKey") + shooterSpeedOffset));
			}
			else
			{
				if(!funnelState)
				{
					//printf("Funnel up!\n");
					bot->getShooter()->SetSpeed(static_cast<float>(constants->getValueForKey("shooterFender") + shooterSpeedOffsetFender));
				}
				else
				{
					bot->getShooter()->SetSpeed(static_cast<float>(constants->getValueForKey("shooterFenderArmDown") + shooterSpeedOffsetFenderFunnel));
				}
			}
			
		} 
		else 
		{
			bot->getShooter()->SetSpeed(0);
		}
	//}
	//previousShooter = cb->getOperatorButton(4);
	
	if(cb->getButtonRampManipulator())
	{
		if(RMPState)
			RMPState = false;
		else
		{
			RMPState = true;
		}	
	}
	
	
	if(RMPState)
		bot->getRampManip()->Set(true);
	else
		bot->getRampManip()->Set(false);
	
	// Drive shifting
	if( cb->getButtonShifter() )
		bot->askForShift(RAWCRobot::SHIFTER_POS_HIGH);
	else
		bot->askForShift(RAWCRobot::SHIFTER_POS_LOW);
	
	if(!cb->getSteeringButton(3))
	{
		bot->driveSpeedTurn(cb->getDriveStickY(), cb->getSteeringX(), cb->getSteeringButton(FAST_TURN));
	}
	else
	{
		bot->cameraPID(cb->getDriveStickY());
	}
	
	//bot->getShooter()->SetRaw(cb->getOperatorY());
	
	//printf("running\n");
	if(cb->getOperatorButton(5))
	{
		//printf("okay\n");
		bot->getIntake()->Set(Relay::kForward);
	} else if(cb->getOperatorButton(7))
	{
		bot->getIntake()->Set(Relay::kReverse);
	} else {
		bot->getIntake()->Set(Relay::kOff);
	}
	
	if(cb->getOperatorButton(6))
	{
		bot->getChute()->Set(Relay::kForward);
	} else if(cb->getOperatorButton(8))
	{
		bot->getChute()->Set(Relay::kReverse);
	} else {
		bot->getChute()->Set(Relay::kOff);
	}
}

