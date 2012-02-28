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
	case amLeft1:
		sprintf(str,"Left Lane - 1 Tube    ");
		s.assign(str);
		break;
	case amLeft2:
		sprintf(str,"Left Lane - 2 Tube    ");
		s.assign(str);
		break;
	case amMiddle1:
		sprintf(str,"Middle Lane - 1 Tube  ");
		s.assign(str);
		break;
	case amMiddle2:
		sprintf(str,"Middle Lane - 2 Tube  ");
		s.assign(str);
		break;
	case amMiddle3:
		sprintf(str,"Middle Lane - 3 Tube  ");
		s.assign(str);
		break;
	case amRight1:
		sprintf(str,"Right Lane - 1 Tube   ");
		s.assign(str);
		break;
	case amRight2:
		sprintf(str,"Right Lane - 2 Tube   ");
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
	
	//0.0429296875
	case amLeft1:
		/*
		autoController->addCommand(CMD_DRIVE_DIST, 1000, 0, ARM_TOP,5);
		autoController->addCommand(CMD_TELESCOPE, 0, 0, ARM_TOP, 0.1);
		autoController->addCommand(CMD_DRIVE_DIST, 5000, 0, ARM_TOP,5);
		autoController->addCommand(CMD_RELEASE_TUBE, 0, 0, ARM_TOP, 0.1);
		autoController->addCommand(CMD_DRIVE_DIST, 2000, 0, ARM_TOP, 5);
		autoController->addCommand(CMD_DRIVE_DIST, 350, 0, ARM_STOP, 5);
		autoController->addCommand(CMD_INTAKE_TUBE, 0, 0, ARM_TOP, 0.5);
		autoController->addCommand(CMD_TURN, 45, 0, ARM_STOP, 0.45);
		autoController->addCommand(CMD_DRIVE_DIST, 4000, 45, ARM_STOP, 1);
		autoController->addCommand(CMD_TURN, 0, 0, ARM_STOP, 0.45);
		autoController->addCommand(CMD_DRIVE_DIST, 4900, 0, ARM_TOP, 5);
		autoController->addCommand(CMD_RELEASE_TUBE, 0, 0, ARM_TOP, 0.1);
		autoController->addCommand(CMD_DRIVE_DIST, 2000, 0, ARM_STOP, 5);
		autoController->addCommand(CMD_RELEASE_TUBE, 0, 0, ARM_TOP, 0.1);
		autoController->addCommand(CMD_DRIVE_DIST, 1500, 0, ARM_STOP, 5);
		*/
		break;
	case amLeft2:
		/*
		autoController->addCommand(CMD_DRIVE_DIST, 5000, 0, 5);
		autoController->addCommand(CMD_DRIVE_DIST, 350, 0, 5);
		autoController->addCommand(CMD_TURN, -45, 0, 0.45);
		autoController->addCommand(CMD_DRIVE_DIST, 870, -45, 1);
		autoController->addCommand(CMD_TURN, 0, 0, 0.45);
		autoController->addCommand(CMD_DRIVE_DIST, 5150, 0, 5);
		autoController->addCommand(CMD_DRIVE_DIST, 2500, 0, 5);
		*/
		printf("Wrote Left1\r\n");
		break;

	case amDoNothing:

		break;
	default:
		index = amFirst + 1;
		break;
	}
	
	
	//TDO: Make this
}
