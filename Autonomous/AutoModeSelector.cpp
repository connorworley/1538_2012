//=============================================================================
// File: AutoModeSelector.cpp
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

#include "AutoModeSelector.h"
#include <string>


// Note:
// This all sucks.
// I want to change it

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
	case amAimAndFire:
		sprintf(str,"Auto-aim and fire     ");
		s.assign(str);
		break;
	case amDoNothing:
		sprintf(str,"Do Nothing (Far Zone) ");
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
	autoController->reset();
	//autoController->addCommand(CMD_WAIT, 0, 0, ARM_STOP, 1);
	switch(index)
	{
	
	case amAimAndFire:
		autoController->addCommand(CMD_AUTOAIM, 0, 0, 4800, 3);
		autoController->addCommand(CMD_CHUTE, 0, 0, 4800, 1);
		break;

	case amDoNothing:

		break;
	default:
		index = amFirst + 1;
		break;
	}
	
	
	//TDO: Make this
}
