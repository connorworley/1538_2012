//=============================================================================
// File: AutoModeController.cpp
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

#include "AutoModeController.h"
#include "../RAWCRobot.h"
#include "../RAWCConstants.h"
#include "../accelFilter.h"
#include <math.h>

AutoModeController* AutoModeController::singletonInstance = NULL;

/// Creates (if needed) and returns a sidngleton Control Board instance
AutoModeController* AutoModeController::getInstance()
{
	if( singletonInstance == NULL)
		singletonInstance = new AutoModeController(); 
	return singletonInstance;
}

AutoModeController::AutoModeController()
:
	previousError(0)
{
	bot = RAWCRobot::getInstance();
	timer = new Timer();
	curCmd = cmdNULL;
	
	extendArm = false;
	
	timer->Start();
	
	reset();
}

Relay::Value toRelayValue(cmdArg val)
{
	if(val == 1)
		return Relay::kForward;
	if(val == -1)
		return Relay::kReverse;
	
	return Relay::kOff;
}

void AutoModeController::addCommand(RobotCommandNames_e cmd, 
					cmdArg arg1, cmdArg arg2, cmdArg arg3, cmdArg arg4, cmdArg arg5, cmdArg arg6, cmdArg arg7)
{
	// Make the new command
	RobotCommand newCmd;
	newCmd.cmd = cmd;
	newCmd.encoderCount = arg1;
	newCmd.heading = arg2;
	newCmd.shooter = arg3;
	newCmd.arm = arg4;
	newCmd.intake = toRelayValue(arg5);
	newCmd.chute = toRelayValue(arg6);
	newCmd.timeout = arg7;
	
	
	// add it to the end of the list
	cmdList.push_back(newCmd);
}

void AutoModeController::reset()
{
	//RAWCConstants * rc = RAWCConstants::getInstance();
	bot->getGyro()->Reset();
	bot->getLeftEncoder()->Reset();
	bot->getFunnel()->Set(false);
	bot->getShooter()->Reset();
	bot->getIntake()->Set(Relay::kOff);
	bot->getChute()->Set(Relay::kOff);
	cmdList.clear();
	curCmd = cmdNULL;
	bot->askForShift(RAWCRobot::SHIFTER_POS_HIGH);
}

bool AutoModeController::handle()
{
	bool result = false;
	bool thisIsNull = false;
	
	// Run the command
	switch(curCmd.cmd)
	{
		case CMD_AUTOAIM:
			result = bot->cameraPID(0);
			bot->getShooter()->SetSpeed(curCmd.shooter);
			bot->getFunnel()->Set(curCmd.arm);
			bot->getIntake()->Set((Relay::Value)curCmd.intake);
			bot->getChute()->Set((Relay::Value)curCmd.chute);
			break;
		case CMD_DRIVE:
			result = driveDistanceWithHeading(curCmd.encoderCount, curCmd.heading);
			bot->getShooter()->SetSpeed(curCmd.shooter);
			bot->getFunnel()->Set(curCmd.arm);
			bot->getIntake()->Set((Relay::Value)curCmd.intake);
			bot->getChute()->Set((Relay::Value)curCmd.chute);
			break;
		case CMD_DRIVE_DIST:
			result = driveDistancePWithHeading(curCmd.encoderCount, curCmd.heading);
			bot->getShooter()->SetSpeed(curCmd.shooter);
			bot->getFunnel()->Set(curCmd.arm);
			bot->getIntake()->Set((Relay::Value)curCmd.intake);
			bot->getChute()->Set((Relay::Value)curCmd.chute);
			break;
		case CMD_TURN:
			result = turnHeading(curCmd.heading);
			bot->askForShift(RAWCRobot::SHIFTER_POS_LOW);
			bot->getShooter()->SetSpeed(curCmd.shooter);
			bot->getFunnel()->Set(curCmd.arm);
			bot->getIntake()->Set((Relay::Value)curCmd.intake);
			bot->getChute()->Set((Relay::Value)curCmd.chute);
			bot->getLeftEncoder()->Reset();

			break;
		case CMD_NULL:
			thisIsNull = true;
			doNothing();
			
			result = true;
			break;
			
		case CMD_WAIT:
			doNothing();
			
			bot->getShooter()->SetSpeed(curCmd.shooter);
			bot->getFunnel()->Set(curCmd.arm);

			
			result = false;
			break;
		
		default :
			doNothing();
			result = true;
			break;
	}
	
	// Check if this command is done
	if(result == true || thisIsNull || timer->Get() > curCmd.timeout){
		// This command is done, go get the next one
		if(cmdList.size() > 0 )
		{
			curCmd = cmdList.front();
			cmdList.pop_front();
			timer->Reset();
		}
		else curCmd = cmdNULL;
	}
	return false;
}

// Drive Functions
void AutoModeController::doNothing()
{
	bot->driveLeftRight(0,0);
	//bot->getArm()->SetMotor(0);
	//bot->getArm()->
}

bool AutoModeController::turnHeading(cmdArg heading)
{
	const float pGain = 1.2;
	
	float currentHeading = bot->getGyro()->GetAngle();
	float turn = heading - currentHeading;
	turn /= 100;
	
	bot->getLeftEncoder()->Reset();
	
	previousError = turn;
	
	//printf("%f\r\n", turn);
	
	float output = (LimitMix(turn) * pGain) + (LimitMix(previousError - turn) * 0.3);
	bot->driveLeftRight(output, -output);
	
	if((currentHeading < heading + 1 && currentHeading > heading - 1))
	{
		printf("Done with distance\r\n");
		return false;
	}	
	else
		return false;
}

bool AutoModeController::driveDistanceWithHeading(cmdArg distance, cmdArg heading)
{
	float distanceP = distance - bot->getLeftEncoder()->GetRaw();
	float turn = heading - bot->getGyro()->GetAngle();
		
	distanceP *= 0.005;
	turn /= 100;
	
	printf("%f, %d\r\n", distanceP, bot->getLeftEncoder()->GetRaw());
		
	bot->driveSpeedTurn(LimitMix(-distanceP) * 0.9, LimitMix(-turn)*1.3, true);
//	bot->driveSpeedTurn(LimitMix(-distanceP) * 1, 0, true);

	return false;
}

bool AutoModeController::driveDistancePWithHeading(cmdArg distance, cmdArg heading)
{
	float distanceP = distance - bot->getLeftEncoder()->GetRaw();
	float currentDistance = bot->getLeftEncoder()->GetRaw();
	float currentHeading = bot->getGyro()->GetAngle();
	float turn = heading - bot->getGyro()->GetAngle();
		
	distanceP *= 0.005;
	turn /= 100;
	
	printf("%f, %d\r\n", distanceP, bot->getLeftEncoder()->GetRaw());
		
	bot->driveSpeedTurn(LimitMix(-distanceP) * 0.9, LimitMix(-turn)*1.3, true);
//	bot->driveSpeedTurn(LimitMix(-distanceP) * 1, 0, true);

	if((currentDistance < distance + 40 && currentDistance > distance - 40) &&
	   (currentHeading < heading + 1 && currentHeading > heading -1))
	{
		printf("Done with distance\r\n");
		return true;
	}	
	else
		return false;
}



