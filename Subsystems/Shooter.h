//=============================================================================
// File: Shooter.h
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

#ifndef __SHOOTER_H__
#define __SHOOTER_H__

#include "WPILib.h"
#include "../CowConstants.h"
#include "../DaisyFilter.h"
#include <pthread.h>


class Shooter
{
	private:
		Victor* motorA;
		Victor* motorB;
		Encoder* encoder;
		Solenoid* hood;
		AnalogChannel* irSensor;
		
		float wantedAccel;
		float wantedSpeed;
		float previousWantedAccel;
		
		float previousError;
		
		float rawValue;
		
		float totalI;
		bool lockI;
		
		float averageAccel;
		float previousAccel;
		
		bool PIDEnabled;
		int counter;
		
		double PID_P;
		
		bool passedLowerLimit;
		bool passedLowerLimitLatch;
		
		CowConstants* constants;
		
		DaisyFilter *shooterLowPass;
		pthread_mutex_t* mutex;
	
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
		void PIDOverride(bool state);
		
		void Reset();
		
		bool ballReady();
		
		void ResetIIRGain();
		
		double sensorPos;
		
		~Shooter();
};
#endif
