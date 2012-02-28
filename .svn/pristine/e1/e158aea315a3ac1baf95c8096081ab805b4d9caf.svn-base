//=============================================================================
// File: Shooter.cpp
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

#include "Shooter.h"
#include "../Declarations.h"
#include "../RAWCLib.h"

using namespace RAWCLib;

Shooter::Shooter(int motorApwm, int motorBpwm, int encoderAchan, int encoderBchan)
:
wantedAccel(0),
wantedSpeed(0),
previousWantedAccel(0),
previousError(0),
totalI(0),
lockI(false),
averageAccel(0),
previousAccel(0),
PIDEnabled(true),
counter(0)
{
	motorA = new Victor(motorApwm);
	motorB = new Victor(motorBpwm);
	encoder = new Encoder(encoderAchan, encoderBchan, false, Encoder::k1X);
	hood = new Solenoid(SHOOTER_SOLENOID_CHAN);
	
	encoder->Start();
}

void Shooter::Handle()
{
	//wantedSpeed = 4000;
	
	double sensorPos = encoder->GetRate();
	
	double delta = sensorPos - previousAccel;
	averageAccel += delta;
	previousAccel = sensorPos;
	
	double PID_P = wantedSpeed - sensorPos;
	double PID_D = previousError - PID_P;
	
	if(!lockI && PID_P > 0)
		totalI += (PID_P > 25) ? 25 : PID_P;
	
	PID_P *= 0.004;
	double PID_I = totalI * 0.00008;
	PID_D *= 0.002;
	
	previousError = PID_P;
		
	float output = VictorLinearize(PID_P + PID_I + PID_D);
	output = LimitMix(output);
	
	SetRaw(output);
	
	static int counter = 0;
	if(counter % 10 == 0)
	{
		averageAccel /= 10;
		
		float band = wantedSpeed * (45.0/6900.0) + 5;
		
		if(!lockI &&
				sensorPos > wantedSpeed - band && sensorPos < wantedSpeed + band &&
				averageAccel > -0.3 && averageAccel < 0.3)
		{
			//printf("\n\nLOCKED I GAIN\n\n\n");
			lockI = true;
		}
		
		//printf("Wanted: %f, Actual: %f, Output: %f, I total: %f\n", wantedSpeed, (float)encoder->GetRate(), output, totalI);
	
		averageAccel = 0;
	}
	counter++;
}

void Shooter::SetAccel(float accel)
{	
	//TODO: check this deadband out
	if(accel < 0.3 && accel > -0.3)
		accel = 0;
	
	this->previousWantedAccel = this->wantedAccel;
	
	this->wantedAccel = accel;
	this->wantedAccel -= this->previousWantedAccel;
	this->wantedSpeed += this->wantedAccel / 100; //TODO: check this 100 constant
}

void Shooter::SetSpeed(const float speed)
{
	lockI = false;
	totalI = 0;
	this->wantedSpeed = speed;
}

void Shooter::Reset()
{
	this->wantedSpeed= 0;
	lockI = false;
	totalI = 0;
	encoder->Reset();
}

void Shooter::SetRaw(float value)
{
	if(value < 0)
		value = 0;
	this->motorA->Set(value);
	this->motorB->Set(-value);
}

bool Shooter::PIDStatus()
{
	return this->PIDEnabled;
}

void Shooter::PIDOverride()
{
	if(PIDStatus())
		this->PIDEnabled = false;
	else
		this->PIDEnabled = true;
}

bool Shooter::GetHoodState()
{
	return hood->Get();
}

void Shooter::ExtendHood()
{
		hood->Set(true);
}

void Shooter::RetractHood()
{
	hood->Set(false);
}
double Shooter::GetCurrentSpeed()
{
	return encoder->GetRate();
}

float Shooter::GetCurrentWantedSpeed()
{
	return this->wantedSpeed;
}

bool Shooter::AtGoalSpeed()
{
	//TODO: find a better way to determine
	return true;
}

Shooter::~Shooter()
{
	delete motorA;
	delete motorB;
	delete encoder;
	delete hood;
}
