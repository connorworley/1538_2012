//=============================================================================
// File: AutoModeController.h
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

#ifndef _AUTO_CONTROLLER_H
#define _AUTO_CONTROLLER_H
#include "../CowRobot.h"
#include "../CowLib.h"
#include "../CowConstants.h"

using namespace CowLib;


// What type of argument?
typedef float cmdArg;

typedef enum RobotCommandNames_e
{
	CMD_NULL = 0,
	CMD_AUTOAIM,
	CMD_DRIVE,
	CMD_DRIVE_DIST,
	CMD_WAIT,
	CMD_TURN
}RobotCommandNames_e;

typedef struct RobotCommand{
	RobotCommandNames_e cmd;
	cmdArg encoderCount;
	cmdArg heading;
	cmdArg shooter;
	cmdArg arm;
	cmdArg intake;
	cmdArg chute;
	cmdArg nBallsWanted;
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
	CowRobot *bot;
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
	void addCommand(RobotCommandNames_e cmd, cmdArg arg1, cmdArg arg2, cmdArg arg3, cmdArg arg4, cmdArg arg5, cmdArg arg6, cmdArg arg7, cmdArg arg8);

	bool handle();
	void reset();
	
};


#endif

