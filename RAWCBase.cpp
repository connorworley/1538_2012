//=============================================================================
// File: RAWCBase.cpp
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

#include "WPILib.h"
#include "RAWCRobot.h"
#include "DashboardDataFormat.h"
#include "Vision/AxisCamera.h"
#include "Controllers/OperatorController.h"
#include "Controllers/AutoModeController.h"
#include "Autonomous/AutoModeSelector.h"
#include "RAWCConstants.h"


// Uncomment this to make the camera work
//#define USE_CAMERA

DriverStationLCD *m_dsLCD;

class RAWCBase : public IterativeRobot
{
	DriverStation *m_ds;
	RAWCRobot *bot;
	OperatorController * opController;
	AutoModeController * autoController;
	AutoModeSelector * autoSelector;
	int autoIndex;
	RAWCConstants * constants;
	
public:
	RAWCBase(void)	{
		
		
		GetWatchdog().SetEnabled(false);
	
		m_dsLCD = DriverStationLCD::GetInstance();
		
		bot = RAWCRobot::getInstance();	
		opController = new OperatorController();
		autoController = AutoModeController::getInstance();
		autoSelector = new AutoModeSelector();
		autoSelector->increment();
//		autoSelector->incrementSecondary();
		
		
		constants = RAWCConstants::getInstance();
		constants->restoreData();
		
		//37 Max
#define COMP_BOT
#ifdef COMP_BOT
		const int zeroPoint = 29;
#else
		const int zeroPoint = 34;
#error
#endif
		
		constants->insertKeyAndValue("shooterKey", 3800);
		constants->insertKeyAndValue("shooterFender", 4800);
		constants->insertKeyAndValue("shooterFenderArmDown", 3000);

		
		constants->save();
		
	}
	void RobotInit(void) {	
	}
	void DisabledInit(void) {
		autoController->reset();
		constants->restoreData();
	
		autoIndex = 1;
	}
	void AutonomousInit(void) {
		// Commit the selected auto mode to the controller
		printf("In auto init\r\n");
		constants->restoreData();
		autoController->reset();
		autoSelector->writeToAutoModeController(autoController);
		RAWCRobot::getInstance()->getLeftEncoder()->Reset();
		RAWCRobot::getInstance()->getGyro()->Reset();
		
		
	}
	void TeleopInit(void) 
	{
		bot->getShooter()->Reset();
		//bot->getArm()->Reset();
	}
	void DisabledContinuous(void) 
	{
		Wait(0.01);
	}
	void AutonomousContinuous(void)	
	{
		Wait(0.01);
	}
	void TeleopContinuous(void) {
		Wait(0.01);
	}
	void DisabledPeriodic(void)  {
		autoController->reset();	
		if( opController->cb->getButtonAutoSelect())
		{
			autoSelector->increment();
		}


		//Print it
		PrintToLCD::print(true, 1, 1, "Auto Mode: ");
		PrintToLCD::print(true, 2, 1, autoSelector->description().c_str());
//		PrintToLCD::print(true, 3, 1, "D:%2f",constants->getValueForKey("KickerPosShort")*10.0/10 );
//		PrintToLCD::print(true, 4, 7, "N:%2f",constants->getValueForKey("KickerPosNear")*10.0/10 );
//		PrintToLCD::print(true, 4, 12, "M:%2f",constants->getValueForKey("KickerPosMiddle")*10.0/10 );
//		PrintToLCD::print(true, 4, 17, "F:%2f", constants->getValueForKey("KickerPosFar")*10.0/10 );
		PrintToLCD::finalizeUpdate();
		//sendIOPortData();
	}
	void AutonomousPeriodic(void) {
		autoController->handle();
		bot->handle();
		
		//sendIOPortData();
	}
	void TeleopPeriodic(void) {
		opController->handle();
		bot->handle();
		
		//sendIOPortData();
	}
	
	
};

START_ROBOT_CLASS(RAWCBase);
