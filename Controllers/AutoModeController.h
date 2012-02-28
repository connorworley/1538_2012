//=============================================================================
// File: AutoModeController.h
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

#ifndef _AUTO_CONTROLLER_H
#define _AUTO_CONTROLLER_H
#include "../RAWCRobot.h"
#include "../RAWCLib.h"

using namespace RAWCLib;


// What type of argument?
typedef float cmdArg;

typedef enum RobotCommandNames_e
{
	CMD_NULL = 0,
	CMD_DRIVE,
	CMD_DRIVE_DIST,
	CMD_RELEASE_TUBE,
	CMD_INTAKE_TUBE,
	CMD_TELESCOPE,
	CMD_WAIT,
	CMD_TURN
}RobotCommandNames_e;

typedef struct RobotCommand{
	RobotCommandNames_e cmd;
	cmdArg encoderCount;
	cmdArg heading;
	cmdArg armPos;
	cmdArg roller;
	cmdArg timeout;
}RobotCommand;

// A dead command for use later
const RobotCommand cmdNULL =
{
	CMD_NULL,
	0,
	0,
	0
};

class AutoModeController
{
private:
	
	static AutoModeController *singletonInstance;
	
	Timer * timer;
	RAWCRobot *bot;
	deque<RobotCommand> cmdList;
	RobotCommand curCmd;
	
	bool extendArm;
	
	float previousError;
	
	// do nothing
	void doNothing();
	bool driveDistanceWithHeading(cmdArg distance, cmdArg heading);
	bool driveDistancePWithHeading(cmdArg distance, cmdArg heading);
	bool turnHeading(cmdArg heading);
public:
	AutoModeController();
	static AutoModeController* getInstance();
	void addCommand(RobotCommandNames_e cmd, cmdArg arg1, cmdArg arg2, cmdArg arg3, cmdArg arg4);

	bool handle();
	void reset();
	
};


#endif

