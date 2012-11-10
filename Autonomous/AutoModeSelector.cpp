//=============================================================================
// File: AutoModeSelector.cpp
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

#include "AutoModeSelector.h"
#include "../CowConstants.h"
#include <string>

void AutoModeSelector::increment()
{
	index++;
	if(index == amLast)
		index = amFirst + 1;
	printf("Auto mode selected: %d\r\n",index);
}

string AutoModeSelector::description()
{
	char str[25];
	memset(str, '.', 25);
	string s = "";

	/*
	amLeft1,
	amLeft2,
	amMiddle1,
	amMiddle2,
	amMiddle3,
	amRight1,
	amRight2,
	
	*/
	
	switch (index)
	{
	case amAimAndFire1s:
		sprintf(str,"Wait 1s and fire      ");
		s.assign(str);
		break;
	case turnTest:
		sprintf(str,"Turn test                ");
		s.assign(str);
		break;
	case amAimAndFire3s:
		sprintf(str,"Wait 3s and fire      ");
		s.assign(str);
		break;
	case amAimAndFire5s:
		sprintf(str,"Wait 5s and fire      ");
		s.assign(str);
		break;
	case amAimAndFire7s:
		sprintf(str,"Wait 7s and fire      ");
		s.assign(str);
		break;
	case amAimAndFire9s:
		sprintf(str,"Wait 9s and fire      ");
		s.assign(str);
		break;
	case amAimAndFire11s:
		sprintf(str,"Wait 11s and fire      ");
		s.assign(str);
		break;
	case amAimAndFire13s:
		sprintf(str,"Wait 13s and fire      ");
		s.assign(str);
		break;
	case amDoNothing:
		sprintf(str,"Do Nothing             s");
		s.assign(str);
		break;
	default:
		sprintf(str,"It broke              ");
		index = amFirst + 1;
		s.assign(str);
		break;
	}

	return s;
}

void AutoModeSelector::writeToAutoModeController(AutoModeController * autoController)
{
	// addCommand(TYPE, DRIVE COUNTS, HEADING, SHOOTER, ARM PISTON STATE, INTAKE, CHUTE, WANTED # BALLS SHOT, TIMEOUT)
	
	autoController->reset();
	//autoController->addCommand(CMD_WAIT, 0, 0, ARM_STOP, 1);
	
	int shooterKey = CowConstants::getInstance()->getValueForKey("shooterKey");
	
	switch(index)
	{
	
	case amAimAndFire1s:
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 0, 0, 1.5);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 1, 0, 3);
		
//		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 1, 1, 0, 3);
//
//		autoController->addCommand(CMD_TURN, 0, 180, 0, 0, 0, 0, 0, 3);
//		autoController->addCommand(CMD_WAIT, 0, 180, 0, 1, 0, 0, 0, 0.2);
//		autoController->addCommand(CMD_DRIVE, 1380, 170, 1, 1, 0, 0, 0, 5);
//		autoController->addCommand(CMD_DRIVE, 0, 170, 1, 1, 0, 0, 0, 5);
//		autoController->addCommand(CMD_TURN, 0, 0, 0, 0, 0, 0, 0, 3);
//		// turn on chute
//		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 1, 1, 2, 3);




		//autoController->addCommand(CMD_AUTOAIM, 0, 0, shooterKey, 3);
//		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 1);
//		autoController->addCommand(CMD_CHUTE, 0, 0, shooterKey, 3);
		break;
	case turnTest:
		autoController->addCommand(CMD_TURN, 0, 90, 0, 0, 0, 0, 0, 3600);
	case amAimAndFire3s:
		//autoController->addCommand(CMD_AUTOAIM, 0, 0, shooterKey, 3);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 0, 0, 3);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 1, 0, 3);
		break;
	case amAimAndFire5s:
		//autoController->addCommand(CMD_AUTOAIM, 0, 0, shooterKey, 3);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 0, 0, 5);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 1, 0, 3);
		break;
	case amAimAndFire7s:
		//autoController->addCommand(CMD_AUTOAIM, 0, 0, shooterKey, 3);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 0, 0, 7);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 1, 0, 3);
		break;
	case amAimAndFire9s:
		//autoController->addCommand(CMD_AUTOAIM, 0, 0, shooterKey, 3);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 0, 0, 9);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 1, 0, 3);
		break;
	case amAimAndFire11s:
		//autoController->addCommand(CMD_AUTOAIM, 0, 0, shooterKey, 3);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 0, 0, 11);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 1, 0, 3);
		break;
	case amAimAndFire13s:
		//autoController->addCommand(CMD_AUTOAIM, 0, 0, shooterKey, 3);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 0, 0, 13);
		autoController->addCommand(CMD_WAIT, 0, 0, shooterKey, 0, 0, 1, 0, 3);
		break;
		
	case amDoNothing:

		break;
	default:
		index = amFirst + 1;
		break;
	}
	
	
	//TDO: Make this
}
