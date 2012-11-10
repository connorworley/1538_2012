#ifndef _Cow_CONTROL_BOARD_H
#define _Cow_CONTROL_BOARD_H
//=============================================================================
// File: CowControlBoard.h
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
#include "CowRobot.h"

#define SHIFTER_BUTTON			4
#define AUTON_BUTTON			1
#define PID_BUTTON				3

#define LEFT_GAMEPAD_X			1
#define LEFT_GAMEPAD_Y			2
#define RIGHT_GAMEPAD_X			3
#define RIGHT_GAMEPAD_Y			4
#define STEERING_X				1

#define FAST_TURN				6 

/// This class offers access to the 2010 specific Cow Control Board
///
class CowControlBoard
{
private:
	DriverStationEnhancedIO * ds;
	static CowControlBoard *singletonInstance;
	bool autoLatch;
	bool alignLatch;
	bool rampLatch;
	bool telescopeLatch;
	
	Joystick* driveStick;
	Joystick* steeringWheel;
	Joystick* operatorPanel;
	
	// Construction
	CowControlBoard();
	
public:
	enum DistanceLEDs{
		LED_DISTANCE_NONE = 0,
		LED_DISTANCE_FAR,
		LED_DISTANCE_MIDDLE,
		LED_DISTANCE_NEAR,
		LED_DISTANCE_ALL
	};
	
	static CowControlBoard* getInstance();
	bool getQuickTurn();
	bool getButtonShifter();
	bool getButtonAutoSelect();
	bool getButtonAlign();
	bool getButtonRampManipulator();
	bool getButtonTelescope();

	
	float getDriveStickY();
	float getSteeringX();
	float getOperatorY();
	float getOperatorDpadX();
	float getOperatorDpadY();
	
	bool getDriveButton(int button);
	bool getSteeringButton(int button);
	bool getOperatorButton(int button);
};

#endif
