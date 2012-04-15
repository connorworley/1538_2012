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
		
		taskDeleteHookAdd((FUNCPTR)&taskDeleteHook);
		
		constants = RAWCConstants::getInstance();
		constants->restoreData();
		
		constants->insertKeyAndValue("shooterDelayMS", 1);
		
		constants->insertKeyAndValue("shooterKey", 3700);
		constants->insertKeyAndValue("shooterFender", 4800);
		constants->insertKeyAndValue("shooterFenderArmDown", 3000);
		
		constants->insertKeyAndValue("wheelNonLineararity", 0.7);
		constants->insertKeyAndValue("sensitivityQuickturn", 1.12);
		constants->insertKeyAndValue("sensitivityWithoutQuickturn", 0.38);
		constants->insertKeyAndValue("speedScaling", 0.04);
		
		constants->insertKeyAndValue("shooterP", 1.8);
		constants->insertKeyAndValue("shooterI", 0.018);
		constants->insertKeyAndValue("shooterD", 2);
		constants->insertKeyAndValue("shooterPLimitI", 0.8);
		constants->insertKeyAndValue("shooterIIncrement", 0.3);
		
		
		constants->insertKeyAndValue("shooterDriveP", 0.09);
		constants->insertKeyAndValue("shooterUpperBand", 50);
		constants->insertKeyAndValue("shooterLowerBand", 50);
		
		constants->insertKeyAndValue("cameraColorLevel", 50);
		constants->insertKeyAndValue("cameraBrightness", 50);
		constants->insertKeyAndValue("cameraCompression", 0);
		constants->insertKeyAndValue("cameraExposurePriority", 1);
		constants->insertKeyAndValue("cameraMaxFPS", 30);
		
		constants->save();
		
		GetWatchdog().SetEnabled(false);
	
		m_dsLCD = DriverStationLCD::GetInstance();
		bot = RAWCRobot::getInstance();	
		opController = new OperatorController();
		autoController = AutoModeController::getInstance();
		autoSelector = new AutoModeSelector();
		autoSelector->increment();
//		autoSelector->incrementSecondary();	
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
		//Wait(0.01);
	}
	void TeleopContinuous(void) {
		//Wait(0.01);
	}
	void DisabledPeriodic(void)  {
		bot->getServer()->handle();
		autoController->reset();	
		if( opController->cb->getButtonAutoSelect())
		{
			autoSelector->increment();
		}


		//Print it
		PrintToLCD::print(true, 1, 1, "Auto Mode: ");
		PrintToLCD::print(true, 2, 1, autoSelector->description().c_str());
		PrintToLCD::print(true, 3, 1, "K:%2f", constants->getValueForKey("shooterKey"));
		PrintToLCD::print(true, 4, 1, "SF:%2f", constants->getValueForKey("shooterFender"));
		PrintToLCD::print(true, 5, 1, "SFA:%2f", constants->getValueForKey("shooterFenderArmDown") );
		PrintToLCD::print(true, 6, 1, "SHOOTER:%2f", bot->getShooter()->GetCurrentWantedSpeed() );
//		PrintToLCD::print(true, 4, 17, "F:%2f", constants->getValueForKey("KickerPosFar")*10.0/10 );
		PrintToLCD::finalizeUpdate();
		//sendIOPortData();
	}
	void AutonomousPeriodic(void) {
		bot->getServer()->handle();
		autoController->handle();
		bot->handle();
		
		PrintToLCD::print(true, 1, 1, "Auto Mode: ");
		PrintToLCD::print(true, 2, 1, autoSelector->description().c_str());
		PrintToLCD::print(true, 3, 1, "K:%2f", constants->getValueForKey("shooterKey"));
		PrintToLCD::print(true, 4, 1, "SF:%2f", constants->getValueForKey("shooterFender"));
		PrintToLCD::print(true, 5, 1, "SFA:%2f", constants->getValueForKey("shooterFenderArmDown") );
		PrintToLCD::print(true, 6, 1, "SHOOTER:%2f", bot->getShooter()->GetCurrentWantedSpeed() );
//		PrintToLCD::print(true, 4, 17, "F:%2f", constants->getValueForKey("KickerPosFar")*10.0/10 );
		PrintToLCD::finalizeUpdate();
		
		//sendIOPortData();
	}
	void TeleopPeriodic(void) {
		bot->getServer()->handle();
		opController->handle();
		bot->handle();
		
		if( opController->cb->getButtonAutoSelect())
		{
			constants->restoreData();
		}
		
		PrintToLCD::print(true, 1, 1, "Auto Mode: ");
		PrintToLCD::print(true, 2, 1, autoSelector->description().c_str());
		PrintToLCD::print(true, 3, 1, "K:%2f", constants->getValueForKey("shooterKey"));
		PrintToLCD::print(true, 4, 1, "SF:%2f", constants->getValueForKey("shooterFender"));
		PrintToLCD::print(true, 5, 1, "SFA:%2f", constants->getValueForKey("shooterFenderArmDown") );
		PrintToLCD::print(true, 6, 1, "SHOOTER:%2f", bot->getShooter()->GetCurrentWantedSpeed() );
//		PrintToLCD::print(true, 4, 17, "F:%2f", constants->getValueForKey("KickerPosFar")*10.0/10 );
		PrintToLCD::finalizeUpdate();
		
		//sendIOPortData();
	}
	
	static STATUS taskDeleteHook(WIND_TCB *pTcb)
	{
		char* name = taskName(pTcb->rtpId->initTaskId);
		
		if(strcmp(name, "FTP Server Connection Thread") == 0)
		{
			printf("FTP task deleted! Reloading constants.\n");
			RAWCConstants::getInstance()->restoreData();
		}
		
		return 0;
	}
};

START_ROBOT_CLASS(RAWCBase);
