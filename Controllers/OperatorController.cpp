//=============================================================================
// File: OperatorController.cpp
//
// COPYRIGHT 2012 Robotics Alliance of the West Coast(Cow)
// All rights reserved.  Cow proprietary and confidential.
//             
// The party receiving this software directly from Cow (the "Recipient")
// may use this software and make copies thereof as reasonably necessary solely
// for the purposes set forth in the agreement between the Recipient and
// Cow(the "Agreement").  The software may be used in source code form
// solely by the Recipient's employees/volunteers.  The Recipient shall have 
// no right to sublicense, assign, transfer or otherwise provide the source
// code to any third party. Subject to the terms and conditions set forth in
// the Agreement, this software, in binary form only, may be distributed by
// the Recipient to its users. Cow retains all ownership rights in and to
// the software.
//
// This notice shall supercede any other notices contained within the software.
//=============================================================================
#include "OperatorController.h"
#include "WPILib.h"
#include "../CowLib.h"
#include "../CowControlBoard.h"
#include "../CowRobot.h"
#include "../Declarations.h"
#include "../CowConstants.h"

// Constructor
// TODO: We might not need to pass in Joysticks, if they come from the ControlBoard
OperatorController::OperatorController()
{
	constants = CowConstants::getInstance();
	bot = CowRobot::getInstance();
	cb = CowControlBoard::getInstance();
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
	previousIntake = false;
	previousExhaust = false;
	intakeState = Relay::kOff;
}

// call this when you want to update the bot from a driver
void OperatorController::handle()
{
	if(cb->getOperatorButton(1) && previousDec != cb->getOperatorButton(1))
	{
		if(!bot->getShooter()->GetHoodState())
		{
			constants->insertKeyAndValue("shooterKey", constants->getValueForKey("shooterKey")-constants->getValueForKey("shooterIncrement"));
			//shooterSpeedOffset -= constants->getValueForKey("shooterIncrement");
		} 
		else 
		{
			if(!funnelState)
			{
				constants->insertKeyAndValue("shooterFender", constants->getValueForKey("shooterFender")-constants->getValueForKey("shooterIncrement"));
				//shooterSpeedOffsetFender -= constants->getValueForKey("shooterIncrement");
			} 
			else 
			{
				constants->insertKeyAndValue("shooterFenderArmDown", constants->getValueForKey("shooterFenderArmDown")-constants->getValueForKey("shooterIncrement"));
				//shooterSpeedOffsetFenderFunnel -= constants->getValueForKey("shooterIncrement");
			}
		}
	}
	previousDec = cb->getOperatorButton(1);
	
	if(cb->getOperatorButton(2) && previousInc != cb->getOperatorButton(2))
	{
		if(!bot->getShooter()->GetHoodState())
		{
			constants->insertKeyAndValue("shooterKey", constants->getValueForKey("shooterKey")+constants->getValueForKey("shooterIncrement"));
			//shooterSpeedOffset += constants->getValueForKey("shooterIncrement");
		} 
		else 
		{
			if(!funnelState)
			{
				constants->insertKeyAndValue("shooterFender", constants->getValueForKey("shooterFender")+constants->getValueForKey("shooterIncrement"));
				//shooterSpeedOffsetFender += constants->getValueForKey("shooterIncrement");
			} 
			else 
			{
				constants->insertKeyAndValue("shooterFenderArmDown", constants->getValueForKey("shooterFenderArmDown")+constants->getValueForKey("shooterIncrement"));
				//shooterSpeedOffsetFenderFunnel += constants->getValueForKey("shooterIncrement");
			}
		}
	}
	previousInc = cb->getOperatorButton(2);
	
	bot->getFunnel()->Set(!cb->getOperatorButton(4));
	
	if(cb->getSteeringButton(1) && previousHood != cb->getSteeringButton(1))
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
	previousHood = cb->getSteeringButton(1);
	
	//if(previousShooter != cb->getOperatorButton(4))
	//{
		if(!cb->getOperatorButton(10))
		{
			if(!bot->getShooter()->GetHoodState())
			{
				bot->getShooter()->SetSpeed(constants->getValueForKey("shooterKey"));
				//bot->getShooter()->SetSpeed(static_cast<float>(constants->getValueForKey("shooterKey") + shooterSpeedOffset));
			}
			else
			{
				if(!funnelState)
				{
					//printf("Funnel up!\n");
					bot->getShooter()->SetSpeed(constants->getValueForKey("shooterFender"));
					//bot->getShooter()->SetSpeed(static_cast<float>(constants->getValueForKey("shooterFender") + shooterSpeedOffsetFender));
				}
				else
				{
					bot->getShooter()->SetSpeed(constants->getValueForKey("shooterFenderArm"));
					//bot->getShooter()->SetSpeed(static_cast<float>(constants->getValueForKey("shooterFenderArmDown") + shooterSpeedOffsetFenderFunnel));
				}
			}
			
		} 
		else 
		{
			bot->getShooter()->SetSpeed(0);
		}
	//}
	//previousShooter = cb->getOperatorButton(4);
	
//	if(cb->getButtonRampManipulator())
//	{
//		if(RMPState)
//			RMPState = false;
//		else
//		{
//			RMPState = true;
//		}	
//	}
//	
//	
//	if(RMPState)
//		bot->getRampManip()->Set(true);
//	else
//		bot->getRampManip()->Set(false);
		
	//bot->getRampManip()->Set(!cb->getDriveButton(3));
	
	if(!cb->getDriveButton(3))
	{
		if(cb->getButtonRampManipulator())
		{
			if(RMPState)
				RMPState = false;
			else
			{
				RMPState = true;
			}	
		}
		
		
		bot->getRampManip()->Set(RMPState);
	}
	
	// Drive shifting
	if( cb->getButtonShifter() )
		bot->askForShift(CowRobot::SHIFTER_POS_HIGH);
	else
		bot->askForShift(CowRobot::SHIFTER_POS_LOW);
	
	if(!cb->getSteeringButton(3))
	{
		bot->cameraReset();
		bot->driveSpeedTurn(cb->getDriveStickY(), cb->getSteeringX(), cb->getSteeringButton(FAST_TURN));
	}
	else
	{
		bot->cameraPID(cb->getDriveStickY());
	}
	
	//bot->getShooter()->SetRaw(cb->getOperatorY());
	
	
	intakeState = Relay::kOff;
	
	if(cb->getOperatorButton(8))
	{
		intakeState = Relay::kForward;
	}
	
	if(cb->getOperatorButton(9))
	{
		intakeState = Relay::kReverse;
	}
	
	bot->getIntake()->Set(intakeState);

	
	if(cb->getOperatorButton(5))
	{
		bot->getChute()->Set(Relay::kForward);
	} else if(cb->getOperatorButton(7))
	{
		bot->getChute()->Set(Relay::kReverse);
	} else {
		bot->getChute()->Set(Relay::kOff);
	}
}

