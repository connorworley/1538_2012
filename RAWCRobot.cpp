//=============================================================================
// File: RAWCRobot.cpp
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

#include "RAWCRobot.h"
#include <math.h>
#include "RAWCConstants.h"
#include "RAWCControlBoard.h"
RAWCRobot* RAWCRobot::singletonInstance = NULL;
#include "CounterBase.h"
#include "RAWCLib.h"

#define DRIVECODELCDDEBUGER 1

using namespace RAWCLib;

/// Creates (if needed) and returns a singleton instance of the RAWCRobot
RAWCRobot * RAWCRobot::getInstance()
{
	// If we have not created a robot yet, do so.
	// If we have created a robot, skip this
	if (singletonInstance == NULL)
	{
		singletonInstance = new RAWCRobot();
	}
	return singletonInstance;
}

/// Constructor for RAWCRobot
RAWCRobot::RAWCRobot()
{
	gyroAngle = 0;
	server = new RAWCServer(55555);
	sd = SmartDashboard::GetInstance();
	
	ballsShot = 0;
	timeSinceLastShot = 0;
	previousChuteIRState = false;
	
	camera = &AxisCamera::GetInstance("10.15.38.11");
	camera->WriteResolution(AxisCamera::kResolution_640x480);
	
	camera->WriteColorLevel((int)RAWCConstants::getInstance()->getValueForKey("cameraColorLevel"));
	camera->WriteBrightness((int)RAWCConstants::getInstance()->getValueForKey("cameraBrightness"));
	camera->WriteCompression((int)RAWCConstants::getInstance()->getValueForKey("cameraCompression"));
	camera->WriteExposurePriority((int)RAWCConstants::getInstance()->getValueForKey("cameraExposurePriority"));
	camera->WriteMaxFPS((int)RAWCConstants::getInstance()->getValueForKey("cameraMaxFPS"));


	threshold = new Threshold(0, 50, 200, 255, 0, 50);

	// Set up drive motors
	rightDriveA = new Victor(RIGHT_DRIVE_PWM_A);
	rightDriveB = new Victor(RIGHT_DRIVE_PWM_B);
	rightDriveC = new Victor(RIGHT_DRIVE_PWM_C);
	leftDriveA = new Victor(LEFT_DRIVE_PWM_A);
	leftDriveB = new Victor(LEFT_DRIVE_PWM_B);
	leftDriveC = new Victor(LEFT_DRIVE_PWM_C);

	//Solenoids
	shifterA = new Solenoid(SHIFTER_SOLENOID_CHAN);
	funnel = new Solenoid(FUNNEL_SOLENOID_CHAN);
	rampManip = new Solenoid(RAMP_SOLENOID_CHAN);
	
	cameraP = 0;
	compressorSignal = new DigitalInput(COMPRESSOR_SWITCH);
	compressorRelay = new Relay(COMPRESSOR_RELAY, Relay::kForwardOnly);

	// Set up encoders
	leftDriveEncoder = new Encoder(LEFT_ENCODER_A_CHAN, LEFT_ENCODER_B_CHAN,
			true, CounterBase::k1X);
	rightDriveEncoder = new Encoder(RIGHT_ENCODER_A_CHAN, RIGHT_ENCODER_B_CHAN);
	leftDriveEncoder->SetDistancePerPulse(0.1007081038552321);
	leftDriveEncoder->SetReverseDirection(false);
	leftDriveEncoder->Start();
	rightDriveEncoder->Start();
	leftDriveEncoder->Reset();

	//arm = new Arm(ARM_POT_CHAN, ARM_PWM);
	shooter = new Shooter(SHOOTER_PWM_A, SHOOTER_PWM_B, SHOOTER_ENCODER_A_CHAN,
			SHOOTER_ENCODER_B_CHAN);

	intake = new Roller(INTAKE_RELAY_A, INTAKE_RELAY_B);
	chute = new Roller(CHUTE_RELAY_A, CHUTE_RELAY_B);

	gyro = new Gyro(GYRO_IN_CHAN);
	gyro->SetSensitivity(0.0005); //Kiets
	gyro->Reset();

	velTimer = new Timer();
	velTimer->Start();
	
	gyroFiltered = DaisyFilter::SinglePoleIIRFilter(0.2f);

	//pitchGyro = new Gyro(PITCH_GYRO_IN_CHAN);
	//pitchGyro->SetSensitivity(0.004);

	//pitchGyro->SetSensitivity(0.0052083333333333);

	//js = new Joystick(1);
	velocity = 0;
	previous_encoder = 0;

	shifterCounts = 0;
	currentShiftState = SHIFTER_STATE_HIGH;

	wantedUpperStage = 0;
	wantedLowerStage = 0;

	wantedLeftDrive = 0;
	wantedRightDrive = 0;

	previousAngle = 0.0;
	shooterGyroSetPoint = 0;
	gotAngle = false;
	reset = false;
	
	cameraPIDEnable = false;
	cameraInitialAngle = 0;
	
	cameraGetNewImage = true;
	// Initially, we are in drive mode
	setMode(DRIVE_MODE);
}

int RAWCRobot::getBallCount()
{
	return ballsShot;
}

/// Used to handle the recurring logic funtions inside the robot.
/// Please call this once per update cycle.
void RAWCRobot::handle()
{		
	printCount++;
	gyroAngle = gyroFiltered->Calculate(gyro->GetAngle());

	// Default drive
	float tmpLeftMotor = wantedLeftDrive;
	float tmpRightMotor = wantedRightDrive;

	ShifterPositions tmpShiftPos = wantedShifterPosition;
	shift(tmpShiftPos);

	setLeftMotors(tmpLeftMotor);
	setRightMotors(tmpRightMotor);

	// first off, take care of this stuff
	compressorHandle();
	shooter->Handle();
	intake->Handle();
	//shooter->ballReady();
	
	if(previousChuteIRState && !shooter->ballReady() && chute->GetTop() == Relay::kForward)
	{
		printf("Ball shot\n");
		ballsShot++;
		timeSinceLastShot = Timer::GetFPGATimestamp();
	}
	
	previousChuteIRState = shooter->ballReady();
	
	if(!shooter->AtGoalSpeed() && shooter->ballReady() || shooter->GetCurrentWantedSpeed() == 0 && shooter->ballReady() && chute->GetTop() != Relay::kReverse)
		chute->Set(Relay::kOff);
	
	
	if(RAWCControlBoard::getInstance()->getDriveButton(3))
	{
		if(timeSinceLastShot + RAWCConstants::getInstance()->getValueForKey("shooterDelayMS") >= Timer::GetFPGATimestamp())
		{
			//printf("Delaying... %f\n", Timer::GetFPGATimestamp());
			chute->Set(Relay::kOff);
		}
	}
	
	chute->Handle();

	velocity = (this->leftDriveEncoder->GetRaw() - previous_encoder);

	if (velocity < 0)
		velocity = -velocity;

	velocity /= this->velTimer->Get();
	velocity /= 1000;
	this->velTimer->Reset();
	previous_encoder = this->leftDriveEncoder->GetRaw();

	if (printCount % 10 == 0)
	{
		printf("Filt: %f, Gyro: %f\r\n", gyroAngle, gyro->GetAngle());
		//server->print("Working!\n");

	}
	char c[80];
	sprintf((char*)&c, "%d,%d,0,0,0,0", (int)shooter->GetCurrentWantedSpeed(), (int)shooter->GetCurrentSpeed());
	sd->PutString("shooterData", (char*)&c);

}

void RAWCRobot::cameraReset()
{
	cameraGetNewImage = true;
	cameraInitialAngle = gyroAngle;
}

bool RAWCRobot::cameraPID(float y)
{
	if(cameraGetNewImage)
	{
		image = camera->GetImage();
		cameraInitialAngle = gyroAngle;
		
		if(camera->IsFreshImage())
		{
			cameraGetNewImage = false;
			printf("Got new image: %f, %d\n", cameraInitialAngle, image);
		}
		else
		{
			if(image)
				delete image;
		}
	}
	else
	{
		ParticleFilterCriteria2 criteria[] =
		{
		{ IMAQ_MT_BOUNDING_RECT_WIDTH, 30, 400, false, false },
		{ IMAQ_MT_BOUNDING_RECT_HEIGHT, 40, 400, false, false } };
		//image->Write("img.jpg");
		
		if(image)
		{
			//BinaryImage *thresholdImage = image->ThresholdHSL(90, 155, 145, 255, 55, 201);
			BinaryImage *thresholdImage = image->ThresholdRGB(*threshold); // get just the red target pixels
			BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 1); // remove small objects (noise)
			BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false); // fill in partial and full rectangles
			BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 2); // find the rectangles
			vector<ParticleAnalysisReport> *sortedReports = convexHullImage->GetOrderedParticleAnalysisReports(); // get the results
			
			bool foundTwoTargets = false;
			RAWCLib::QSortParticleAnalysisReport(sortedReports->begin(), sortedReports->end());
			
			double xWanted = 0;
			for(unsigned int i = 1; i < sortedReports->size(); i++)
			{
				ParticleAnalysisReport *r0 = &(sortedReports->at(i));
				ParticleAnalysisReport *r1 = &(sortedReports->at(i - 1));
				
				if(r0->center_mass_y_normalized > r1->center_mass_y_normalized + 0.25 &&
				   r0->center_mass_y_normalized < r1->center_mass_y_normalized - 0.25)
				{
					xWanted = (r0->center_mass_x_normalized + r1->center_mass_x_normalized)/2.0;
					foundTwoTargets = true;
					break;
				}
			}
			
			for(unsigned int i = 0; i < sortedReports->size(); i++)
			{
				ParticleAnalysisReport *r0 = &(sortedReports->at(i));
				printf("particle: %d  center_mass_x: %d, center_mass_y: %d, %f, %f\n", i, r0->center_mass_y, r0->center_mass_y, r0->center_mass_x_normalized, r0->center_mass_y_normalized);
			}

			
			
			if(foundTwoTargets)
			{
				cameraP = 23.5 * (xWanted);
				//PID_P = RAWCLib::LimitMix(PID_P*1.35, 0.4);
				
				//this->driveSpeedTurn(y, -PID_P, true);
				printf("Using two targets: %f\n", cameraP);
				//image->Write("unfiltered.jpg");
				//convexHullImage->Write("img.jpg");
			}
			else
			{
				if(sortedReports->size() > 0)
				{
					ParticleAnalysisReport *r0 = &(sortedReports->at(sortedReports->size() - 1));
					cameraP = (23.5 * r0->center_mass_x_normalized);
				
//				PID_P = RAWCLib::LimitMix(PID_P*0.6, 0.4);
				//this->driveSpeedTurn(y, -PID_P, true);
				printf("Using one targets %f\n", cameraP);
				}
				//image->Write("unfiltered.jpg");
				//convexHullImage->Write("img.jpg");
			}
				
			delete sortedReports;
			delete filteredImage;
			delete convexHullImage;
			delete bigObjectsImage;
			delete thresholdImage;
			delete image;
		}
	}
	
	printf("PID: %f\n", cameraP);
	return false;
}

/// Allows skid steer robot to be driven using tank drive style inputs
/// @param leftDriveValue
/// @param rightDriveValue
///
void RAWCRobot::driveLeftRight(float leftDriveValue, float rightDriveValue)
{
	wantedLeftDrive = leftDriveValue;
	wantedRightDrive = rightDriveValue;
}

void RAWCRobot::driveSpeedTurn(float speed, float turn, bool quickTurn)
{
	//Linear degredation of steeering based off of velocity
	//	velocity *= 0.003;
	float temp_vel = speed;
	float sensitivity = 0;
	float unscaled_turn = 0;

	if (temp_vel < 0)
		temp_vel = -temp_vel;

	//printf("Velocity: %f, stick: %f\r\n", velocity, temp_vel);
	
	if(speed < 0.10 && speed > -0.10)
		speed = 0;
	if (turn < 0.10 && turn > -0.10 || (speed == 0 && !quickTurn))
		turn = 0;

	unscaled_turn = turn;
	
	
	double nonlinearity_wheel = RAWCConstants::getInstance()->getValueForKey("wheelNonLineararity");
	turn = sin(3.14 / 2.0 * nonlinearity_wheel * turn) / sin(3.14 / 2.0 * nonlinearity_wheel);
	turn = sin(3.14 / 2.0 * nonlinearity_wheel * turn) / sin(3.14 / 2.0 * nonlinearity_wheel);
	//turn = turn * (temp_vel - 0.75);


	if (quickTurn)
		sensitivity = RAWCConstants::getInstance()->getValueForKey("sensitivityQuickturn");
	else
		sensitivity = RAWCConstants::getInstance()->getValueForKey("sensitivityWithoutQuickturn");

	turn *= sensitivity;
	turn = -turn;

	//turn = steeringAngle;
	
	if(!quickTurn)
	{
		turn = turn + (speed * RAWCConstants::getInstance()->getValueForKey("speedScaling"));
	}
	
	static int counter = 0;
	counter++;
	
	//if(counter % 10 == 0)
	//	printf("SPEED: %f, QT: %f, WT: %f\n", speed, quickTurn, (this->getShooter()->GetCurrentWantedSpeed() > 0));
//	if((speed == 0) && (turn == 0) && !quickTurn && (this->getShooter()->GetCurrentWantedSpeed() > 0))
//	{
//		if(!gotAngle)
//		{
//			shooterGyroSetPoint = this->getGyro()->GetAngle();
//			gotAngle = true;
//		}
//			
//		double PV = this->getGyro()->GetAngle();
//		
//		double P = shooterGyroSetPoint - PV;
//		
//		turn = P * RAWCConstants::getInstance()->getValueForKey("shooterDriveP");;
//		
//		if(counter % 10 == 0)
//			printf("SP: %f, PV: %f, P: %f\n", shooterGyroSetPoint, PV, P);
//	}
//	else
//	{
//		gotAngle = false;
//	}

	float left_power = LimitMix(speed + turn);
	float right_power = LimitMix(speed - turn);

	float right_power_real = VictorLinearize(right_power);
	float left_power_real = VictorLinearize(left_power);

	driveLeftRight(left_power_real, right_power_real);
}

/// Allows robot to spin in place
/// @param turnRate
///
void RAWCRobot::quickTurn(float turnRate)
{
	//when provided with + turn, quick turn right

	float left = -1 * turnRate;
	float right = turnRate;

	driveLeftRight(left, right);
}

/// Handles the autmatic turn on/off of the compressor
void RAWCRobot::compressorHandle()
{
	if (!compressorSignal->Get())
	{
		compressorRelay->Set(Relay::kOn);
	}
	else
		compressorRelay->Set(Relay::kOff);
}

/// Returns the value of the drive's left side encoder
Encoder * RAWCRobot::getLeftEncoder()
{
	return leftDriveEncoder;
}

/// Returns the value of the drive's right side encoder
Encoder * RAWCRobot::getRightEncoder()
{
	return rightDriveEncoder;
}

Gyro * RAWCRobot::getGyro()
{
	return gyro;
}

Shooter* RAWCRobot::getShooter()
{
	return shooter;
}

Roller* RAWCRobot::getIntake()
{
	return intake;
}

Roller* RAWCRobot::getChute()
{
	return chute;
}

Solenoid* RAWCRobot::getFunnel()
{
	return funnel;
}

Solenoid* RAWCRobot::getRampManip()
{
	return rampManip;
}

/// Shifts drive gears. 
/// @param shifterPosition
void RAWCRobot::shift(ShifterPositions shifterPosition)
{
	bool solA = false, solB = false;
	ShifterStates nextShiftState = currentShiftState;
	// It takes lots of logic to shift this year
	switch (currentShiftState)
	{
	case SHIFTER_STATE_HIGH:
		solA = false;
		solB = false;
		shifterCounts = 0;
		if (shifterPosition == SHIFTER_POS_LOW)
			nextShiftState = SHIFTER_STATE_LOW;
		break;

	case SHIFTER_STATE_LOW:
		solA = true;
		shifterCounts = 0;
		if (shifterPosition == SHIFTER_POS_HIGH)
			nextShiftState = SHIFTER_STATE_HIGH;
		break;
	default:
		nextShiftState = SHIFTER_STATE_HIGH;
		break;
	}

	shifterA->Set(solA);

	if (currentShiftState != nextShiftState)
	{
		leftDriveA->Set(0);
		leftDriveB->Set(0);
		leftDriveC->Set(0);
		rightDriveA->Set(0);
		rightDriveB->Set(0);
		rightDriveC->Set(0);
		Wait(0.125);
	}

	currentShiftState = nextShiftState;
}

void RAWCRobot::askForShift(ShifterPositions shifterPosition)
{
	wantedShifterPosition = shifterPosition;
}

/// Sets the robot mode
void RAWCRobot::setMode(RobotModes newMode)
{
	mode = newMode;
}

/// sets the left side motors
void RAWCRobot::setLeftMotors(float val)
{
	val = -1.0 * val;

	if (val > 1.0)
		val = 1.00;
	if (val < -1.0)
		val = -1.0;

	leftDriveA->Set(-val);
	leftDriveB->Set(-val);
	leftDriveC->Set(-val);
}

/// sets the left side motors
void RAWCRobot::setRightMotors(float val)
{
	if (val > 1.0)
		val = 1.00;
	if (val < -1.0)
		val = -1.0;

	rightDriveA->Set(-val);
	rightDriveB->Set(-val);
	rightDriveC->Set(-val);
}

bool RAWCRobot::inHighGear()
{
	return (currentShiftState == SHIFTER_STATE_HIGH);
}

RAWCServer* RAWCRobot::getServer()
{
	return server;
}
