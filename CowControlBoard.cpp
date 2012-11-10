//=============================================================================
// File: CowControlBoard.cpp
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
#include "CowControlBoard.h"
#include "WPILib.h"
#include "CowRobot.h"

// Initialize the Singleton instance
CowControlBoard* CowControlBoard::singletonInstance = NULL;

/// Creates (if needed) and returns a singleton Control Board instance
CowControlBoard* CowControlBoard::getInstance()
{
	if( singletonInstance == NULL)
		singletonInstance = new CowControlBoard(); 
	return singletonInstance;
}

/// Constructor for Cow Control Board
CowControlBoard::CowControlBoard()
{
	ds = & DriverStation::GetInstance()->GetEnhancedIO();
	driveStick = new Joystick(1);
	steeringWheel = new Joystick(2);
	operatorPanel = new Joystick(3);
	autoLatch = false;
	alignLatch = false;
	rampLatch = false;
	telescopeLatch = false;
}

/*/// Returns the unscaled throttle joystick value
/// @return Unscaled Throttle Value
///
double CowControlBoard::getRawThrottle(){
	return ds->GetAnalogIn(THROTTLE_CHAN);
}

/// Returns the unscaled wheel joystick value
/// @return Unscaled Wheel Value
///
double CowControlBoard::getRawWheel(){
	return ds->GetAnalogIn(WHEEL_CHAN);
}
*/
/*/// Returns the scaled (-1.0 to 1.0) throttle joystick value
/// @return Scaled Throttle Value
///
float CowControlBoard::getThrottle(){
	return CowLib::AnalogInScale(getRawThrottle(), THROTTLE_CENTER_VALUE);
}

/// Returns the scaled (-1.0 to 1.0) wheel joystick value
/// @return Scaled Wheel Value
///
float CowControlBoard::getWheel(){
	return CowLib::AnalogInScale(getRawWheel(), WHEEL_CENTER_VALUE);
}
*/

/// Returns state of shifter switch
bool CowControlBoard::getButtonShifter()
{
	return getDriveButton(SHIFTER_BUTTON);
}

/// Returns state of autonomous select button
bool CowControlBoard::getButtonAutoSelect()
{
	// This will latch on a press and only return true once per press
	// Active Low signal
	if(getDriveButton(AUTON_BUTTON))
	{
		if(!autoLatch)
		{
			autoLatch = true;
			return true;
		}
	}
	else
		autoLatch = false;
	return false;
}

bool CowControlBoard::getButtonRampManipulator()
{
	// This will latch on a press and only return true once per press
	// Active Low signal
	if(getSteeringButton(4))
	{
		if(!rampLatch)
		{
			rampLatch = true;
			return true;
		}
	}
	else
		rampLatch = false;
	return false;
}

bool CowControlBoard::getButtonAlign()
{
	// This will latch on a press and only return true once per press
	// Active Low signal
	if(steeringWheel->GetRawButton(ALIGN))
	{
		if(!alignLatch)
		{
			alignLatch = true;
			return true;
		}
	}
	else
		alignLatch = false;
	return false;
}

bool CowControlBoard::getButtonTelescope()
{
	// This will latch on a press and only return true once per press
	// Active Low signal
	if(operatorPanel->GetRawButton(TELESCOPE))
	{
		if(!telescopeLatch)
		{
			telescopeLatch = true;
			return true;
		}
	}
	else
		telescopeLatch = false;
	return false;
}

/// Returns the state of the quick turn
bool CowControlBoard::getQuickTurn()
{
	return !getDriveButton(8);
}

float CowControlBoard::getDriveStickY()
{
	return driveStick->GetRawAxis(LEFT_GAMEPAD_Y);
}

float CowControlBoard::getSteeringX()
{
	return steeringWheel->GetRawAxis(STEERING_X);
}

float CowControlBoard::getOperatorY()
{
	return operatorPanel->GetRawAxis(LEFT_GAMEPAD_Y);
}

float CowControlBoard::getOperatorDpadX()
{
	return operatorPanel->GetRawAxis(5);
}

float CowControlBoard::getOperatorDpadY()
{
	return operatorPanel->GetRawAxis(6);
}

bool CowControlBoard::getDriveButton(const int button)
{
	return driveStick->GetRawButton(button);
}

bool CowControlBoard::getOperatorButton(const int button)
{
	return operatorPanel->GetRawButton(button);
}

bool CowControlBoard::getSteeringButton(const int button)
{
	return steeringWheel->GetRawButton(button);
}


