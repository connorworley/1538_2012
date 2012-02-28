//=============================================================================
// File: Shooter.h
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

#ifndef __SHOOTER_H__
#define __SHOOTER_H__

#include "WPILib.h"

class Shooter
{
	private:
		Victor* motorA;
		Victor* motorB;
		Encoder* encoder;
		Solenoid* hood;
		
		float wantedAccel;
		float wantedSpeed;
		float previousWantedAccel;
		
		float previousError;
		
		float totalI;
		bool lockI;
		
		float averageAccel;
		float previousAccel;
		
		bool PIDEnabled;
		int counter;
		
		bool passedLowerLimit;
		bool passedLowerLimitLatch;
	
	public:
		Shooter(int motorApwm, int motorBpwm, int encoderAchan, int encoderBchan);
		void SetAccel(float accel);
		void Handle();
		void SetSpeed(const float speed);
		void SetRaw(float value);
		float GetCurrentWantedSpeed();
		double GetCurrentSpeed();
		
		bool AtGoalSpeed();
		
		bool GetHoodState();
		
		void ExtendHood();
		void RetractHood();
		
		bool PIDStatus();
		void PIDOverride();
		
		void Reset();
		
		~Shooter();
};
#endif
