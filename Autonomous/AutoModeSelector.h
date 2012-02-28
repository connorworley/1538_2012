//=============================================================================
// File: AutoModeSelector.h
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

#ifndef _AUTO_SELECTOR_H
#define _AUTO_SELECTOR_H

#include <string>
#include "../Controllers/AutoModeController.h"

using namespace std;


// Note:
// This all sucks.
// I want to change it

class AutoModeSelector
{
	
public:
	enum AutoModes{
		amFirst = 0,
		amLeft1,
		amLeft2,
		amMiddle1,
		amMiddle2,
		amMiddle3,
		amRight1,
		amRight2,
		amDoNothing,
		amLast
	};	
	
	void increment();
	
	// Get a description of this auto mode
	string description();
	void writeToAutoModeController(AutoModeController * ac);
	
	
private:
	int index;
	int secIndex;
};

#endif
