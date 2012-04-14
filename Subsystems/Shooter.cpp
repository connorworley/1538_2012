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
#include "../RAWCControlBoard.h"
#include "../RAWCConstants.h"

using namespace RAWCLib;

Shooter::Shooter(int motorApwm, int motorBpwm, int encoderAchan, int encoderBchan)
:
wantedAccel(0),
wantedSpeed(0),
previousWantedAccel(0),
previousError(0),
rawValue(0),
totalI(0),
lockI(false),
averageAccel(0),
previousAccel(0),
PIDEnabled(true),
counter(0),
PID_P(0)
{
	constants = RAWCConstants::getInstance();
	motorA = new Victor(motorApwm);
	motorB = new Victor(motorBpwm);
	encoder = new Encoder(encoderAchan, encoderBchan, false, Encoder::k1X);
	encoder->SetMaxPeriod(1.0);
	hood = new Solenoid(SHOOTER_SOLENOID_CHAN);
	
	irSensor = new AnalogChannel(1);
	
	encoder->Start();
}

void Shooter::Handle()
{
	//wantedSpeed = 4000;
	
	double sensorPos = encoder->GetRate();	
	double delta = sensorPos - previousAccel;
	averageAccel += delta;
	previousAccel = sensorPos;
	
	PID_P = wantedSpeed - sensorPos;
	
	PID_P /= 1000.0;
	PID_P *= constants->getValueForKey("shooterP");
	
	if(!lockI && PID_P > 0)
	{
		if(PID_P > 0.3)
			totalI += 0.3;
		else
			totalI += PID_P;
	}
	
	double PID_I = totalI * constants->getValueForKey("shooterI");
	
	
	double PID_D = previousError - PID_P;
	PID_D *= constants->getValueForKey("shooterD");
	
	previousError = PID_P;
		
	float output = VictorLinearize(PID_P + PID_I + PID_D);
	output = LimitMix(output);
	
	SetRaw(output);
	

	
	static int counter = 0;
	if(counter % 5 == 0)
	{
		
		averageAccel /= 10;
		
		if((sensorPos > wantedSpeed - 50) && (sensorPos < wantedSpeed + 50) &&
			(averageAccel > -0.3 && averageAccel < 0.3))
		{
			//printf("\n\nLOCKED I GAIN\n\n\n");
			lockI = true;
		}

		//printf("W: %f, A: %f, O: %f, P: %f, I: %f, D: %f, AC: %f\n", wantedSpeed, (float)encoder->GetRate(), output, PID_P, PID_I, PID_D, averageAccel);
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
	if(this->wantedSpeed != speed)
	{
		lockI = false;
		totalI = 0;
	}
	this->wantedSpeed = speed;
}

void Shooter::Reset()
{
	this->wantedSpeed= 0;
	this->hood->Set(false);
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
	this->rawValue = value;
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
	
	
	//printf("Goal Speed: %f, %f\r\n", PID_P, irSensor->GetVoltage());
//	if(PID_P > -1.1 && PID_P < 1.1)
//		printf("At goal speed\r\n");
//	else
//		printf("At goal speed\r\n");
	
	//printf("IR Sensor: %f\n", irSensor->GetVoltage());
	//printf("Raw Value: %f\n", this->rawValue);
	double upperBand = this->wantedSpeed + RAWCConstants::getInstance()->getValueForKey("shooterUpperBand");
	double lowerBand = this->wantedSpeed - RAWCConstants::getInstance()->getValueForKey("shooterLowerBand");
	
	return (this->GetCurrentSpeed() < upperBand && this->GetCurrentSpeed() > lowerBand);
}

bool Shooter::ballReady()
{
	if(RAWCControlBoard::getInstance()->getDriveButton(3))
		return (irSensor->GetVoltage() < 2.0);
	
	return false;
	
	//printf("IR Sensor: %f\n", irSensor->GetVoltage());
	//return false;
}

Shooter::~Shooter()
{
	delete motorA;
	delete motorB;
	delete encoder;
	delete hood;
}
