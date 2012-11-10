//=============================================================================
// File: Roller.h 
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

#ifndef __ROLLER_H__
#define __ROLLER_H__

#include "WPILib.h"
#include "../Declarations.h"

class Roller
{
private:
	Relay* motorTop;
	Relay* motorBot;
	
	Relay::Value wantedTopMotor;
	Relay::Value wantedBotMotor;
	
public:
	
	Roller(int TopMotor, int BottomMotor);
	~Roller();
	void Handle();
	void Set(Relay::Value val);
	Relay::Value GetTop();
	Relay::Value GetBottom();
	
};

#endif
