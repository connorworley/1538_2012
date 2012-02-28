//=============================================================================
// File: Roller.cpp
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

#include "Roller.h"

Roller::Roller(int TopMotor, int BottomMotor)
{
	this->motorTop = new Relay(TopMotor, Relay::kBothDirections);
	this->motorBot = new Relay(BottomMotor, Relay::kBothDirections);
}

void Roller::Handle()
{
	this->motorTop->Set(wantedTopMotor);
	this->motorBot->Set(wantedBotMotor);
}

Roller::~Roller()
{
	delete this->motorTop;
	delete this->motorBot;
}

void Roller::Set(Relay::Value val)
{
	this->wantedTopMotor = val;
	this->wantedBotMotor = val;
}
