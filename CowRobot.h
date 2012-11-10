#ifndef _Cow_ROBOT_H
#define _Cow_ROBOT_H
//=============================================================================
// File: CowRobot.h
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

#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"

#include "SmartDashboard/SmartDashboard.h"

#include "CowLib.h"
#include "subsystems/Shooter.h"
#include "subsystems/Roller.h"

#include "server/CowServer.h"

#include "taskHookLib.h"
#include "private/rtpLibP.h"

#include "DaisyFilter.h"

/// This class provides core robot functionality and encapsulates 
/// motors, sensors, relays, etc.
///
class CowRobot
{
	public: 
	static CowRobot * singletonInstance;
	
	enum ShifterPositions{
		SHIFTER_POS_LOW = 0,
		SHIFTER_POS_HIGH
	};
	
	enum RobotModes{
		DRIVE_MODE
	};
	
	enum ShifterStates{
		SHIFTER_STATE_HIGH,
		SHIFTER_STATE_LOW
	};
	
	static CowRobot* getInstance();
	
	void driveSpeedTurn(float speed, float turn, bool quickTurn);
	void driveLeftRight(float leftDriveValue, float rightDriveValue);
	void quickTurn(float turn);
	bool cameraPID(float y);
	void compressorHandle();
	Encoder * getLeftEncoder();
	Encoder * getRightEncoder();
	
	Gyro * getGyro();
	Joystick * js;
	Shooter* getShooter();
	Roller* getIntake();
	Roller* getChute();
	Solenoid* getFunnel();
	Solenoid* getRampManip();
	
	Timer* velTimer;	
	void handle();

	
	void askForShift(ShifterPositions shifterPosition);
	
	int getBallCount();
	
	bool inHighGear();
	void cameraReset();
	
	CowServer* getServer();

	
private:
	CowServer* server;
	
	SmartDashboard* sd;
	
	int ballsShot;
	
	float velocity;
	double previous_encoder;
	
	void shift(ShifterPositions shifterPosition);
	void setMode(RobotModes newMode);
	
	// Drive Motors
	Victor *rightDriveA;
	Victor *rightDriveB;
	Victor *rightDriveC;
	Victor *leftDriveA;
	Victor *leftDriveB;
	Victor *leftDriveC;
	
	// Drive shifting pistons
	Solenoid *shifterA;
	Solenoid *funnel;
	Solenoid *rampManip;

	
	// Compressor
	DigitalInput *compressorSignal;

	Relay *compressorRelay;
	Shooter* shooter;
	Roller* intake;
	Roller* chute;
		
	//Sensors
	Gyro * gyro;
	Encoder *leftDriveEncoder;
	Encoder *rightDriveEncoder;

	AxisCamera *camera;
	ColorImage *image;
	
	Threshold *threshold;
	RobotModes mode;
	
	DaisyFilter* gyroFiltered;
	float gyroAngle;
	float cameraP;
	
	double cameraInitialAngle;
	
	ShifterPositions wantedShifterPosition;

	float wantedUpperStage;
	float wantedLowerStage;
	
	float wantedLeftDrive;
	float wantedRightDrive;
	
	float previousAngle;
	bool cameraPIDEnable;
	bool cameraGetNewImage;
	int printCount;
	
	double timeSinceLastShot;
	bool previousChuteIRState;
	
	int shifterCounts;
	
	ShifterStates currentShiftState;
	
	bool gotAngle;
	double shooterGyroSetPoint;
	
	bool reset;
	
	CowRobot();
	
	void setLeftMotors(float val);
	void setRightMotors(float val);
};

#endif
