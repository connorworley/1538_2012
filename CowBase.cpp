//=============================================================================
// File: CowBase.cpp
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
#include "DashboardDataFormat.h"
#include "Vision/AxisCamera.h"
#include "Controllers/OperatorController.h"
#include "Controllers/AutoModeController.h"
#include "Autonomous/AutoModeSelector.h"
#include "CowConstants.h"

static bool wroteOnce = false;


#include <sys/stat.h>

// Uncomment this to make the camera work
//#define USE_CAMERA

time_t constantsLastModified;

DriverStationLCD *m_dsLCD;

class CowBase : public IterativeRobot
{
	DriverStation *m_ds;
	CowRobot *bot;
	OperatorController * opController;
	AutoModeController * autoController;
	AutoModeSelector * autoSelector;
	int autoIndex;
	CowConstants * constants;
	Timer* fieldTime;
	
public:
	CowBase(void)	{
		taskDeleteHookAdd((FUNCPTR)&taskDeleteHook);
		
		struct stat data;
		stat("constants.csv", &data);
		constantsLastModified = data.st_mtime;
		
		fieldTime = new Timer();
		SetPeriod((1.0/200.0));

		constants = CowConstants::getInstance();
		constants->restoreData();
		
		constants->insertKeyAndValue("shooterDelayMS", 1);
		
		constants->insertKeyAndValue("shooterKey", 3750);
		constants->insertKeyAndValue("shooterFender", 4800);
		constants->insertKeyAndValue("shooterFenderArmDown", 3000);
		
		constants->insertKeyAndValue("wheelNonLineararity", 0.7);
		constants->insertKeyAndValue("sensitivityQuickturn", 1.12);
		constants->insertKeyAndValue("sensitivityWithoutQuickturn", 0.38);
		constants->insertKeyAndValue("speedScaling", 0.04);
		
		constants->insertKeyAndValue("shooterIIR", 0.09);
		constants->insertKeyAndValue("shooterP", 0.62);
		constants->insertKeyAndValue("shooterI", 0);
		constants->insertKeyAndValue("shooterD", 0.9);
		constants->insertKeyAndValue("shooterPLimitI", 0.8);
		constants->insertKeyAndValue("shooterIIncrement", 0.3);
		constants->insertKeyAndValue("shooterFF", 0.82);
		constants->insertKeyAndValue("bangBang", 0);
		constants->insertKeyAndValue("shooterIncrement", 50);

		
		constants->insertKeyAndValue("shooterDriveP", 0.09);
		constants->insertKeyAndValue("shooterUpperBand", 60);
		constants->insertKeyAndValue("shooterLowerBand", 60);
		
		constants->insertKeyAndValue("cameraColorLevel", 100);
		constants->insertKeyAndValue("cameraBrightness", 0);
		constants->insertKeyAndValue("cameraCompression", 0);
		constants->insertKeyAndValue("cameraExposurePriority", 1);
		constants->insertKeyAndValue("cameraMaxFPS", 30);
		
		constants->insertKeyAndValue("turnP", 0.2);
		constants->insertKeyAndValue("turnD", 0.03);
		
		
		constants->save();
		
		GetWatchdog().SetEnabled(false);
	
		m_dsLCD = DriverStationLCD::GetInstance();
		bot = CowRobot::getInstance();	
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
		
		if(fieldTime->Get() > 115)
		{
			if(!wroteOnce)
			{
				constants->saveDataToFile("LAST_MATCH_CONSTANTS.csv");
				wroteOnce = true;
			}
			
		}
		
		constants->restoreData();
	
		autoIndex = 1;
	}
	void AutonomousInit(void) {
		// Commit the selected auto mode to the controller
		printf("In auto init\r\n");
		constants->restoreData();
		autoController->reset();
		autoSelector->writeToAutoModeController(autoController);
		CowRobot::getInstance()->getLeftEncoder()->Reset();
		CowRobot::getInstance()->getGyro()->Reset();
		
		
	}
	void TeleopInit(void) 
	{
		bot->getShooter()->Reset();
		fieldTime->Start();
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
		//bot->getServer()->handle();
		opController->handle();
		autoController->reset();	
		if( opController->cb->getButtonAutoSelect())
		{
			autoSelector->increment();
		}


		//Print it
		PrintToLCD::print(true, 1, 1, "Auto Mode: ");
		PrintToLCD::print(true, 2, 1, autoSelector->description().c_str());
		PrintToLCD::print(true, 3, 1, "K:%2f", constants->getValueForKey("shooterKey"));
		//PrintToLCD::print(true, 4, 1, "SF:%2f", constants->getValueForKey("shooterFender"));
		PrintToLCD::print(true, 5, 1, "ACTUAL:%2f", bot->getShooter()->GetCurrentSpeed());
		PrintToLCD::print(true, 6, 1, "DESIRED:%2f", bot->getShooter()->GetCurrentWantedSpeed() );
//		PrintToLCD::print(true, 4, 17, "F:%2f", constants->getValueForKey("KickerPosFar")*10.0/10 );
		PrintToLCD::finalizeUpdate();
		//sendIOPortData();
	}
	void AutonomousPeriodic(void) {
		//bot->getServer()->handle();
		autoController->handle();
		bot->handle();
		
		PrintToLCD::print(true, 1, 1, "Auto Mode: ");
		PrintToLCD::print(true, 2, 1, autoSelector->description().c_str());
		PrintToLCD::print(true, 3, 1, "K:%2f", constants->getValueForKey("shooterKey"));
		//PrintToLCD::print(true, 4, 1, "SF:%2f", constants->getValueForKey("shooterFender"));
		PrintToLCD::print(true, 5, 1, "ACTUAL:%2f", bot->getShooter()->GetCurrentSpeed());
		PrintToLCD::print(true, 6, 1, "DESIRED:%2f", bot->getShooter()->GetCurrentWantedSpeed() );
//		PrintToLCD::print(true, 4, 17, "F:%2f", constants->getValueForKey("KickerPosFar")*10.0/10 );
		PrintToLCD::finalizeUpdate();
		
		//sendIOPortData();
	}
	void TeleopPeriodic(void) {
		//bot->getServer()->handle();
		opController->handle();
		bot->handle();
		
		if( opController->cb->getButtonAutoSelect())
		{
			constants->restoreData();
			bot->getShooter()->ResetIIRGain();
		}
		
		PrintToLCD::print(true, 1, 1, "Auto Mode: ");
		PrintToLCD::print(true, 2, 1, autoSelector->description().c_str());
		PrintToLCD::print(true, 3, 1, "K:%2f", constants->getValueForKey("shooterKey"));
		//PrintToLCD::print(true, 4, 1, "SF:%2f", constants->getValueForKey("shooterFender"));
		PrintToLCD::print(true, 5, 1, "ACTUAL:%2f", bot->getShooter()->GetCurrentSpeed());
		PrintToLCD::print(true, 6, 1, "DESIRED:%2f", bot->getShooter()->GetCurrentWantedSpeed() );
//		PrintToLCD::print(true, 4, 17, "F:%2f", constants->getValueForKey("KickerPosFar")*10.0/10 );
		PrintToLCD::finalizeUpdate();
		
		if(fieldTime->Get() > 115)
		{
			if(!wroteOnce)
			{
				constants->saveDataToFile("LAST_MATCH_CONSTANTS.csv");
				wroteOnce = true;
			}
			
		}
		
		//sendIOPortData();
	}
	
	static STATUS taskDeleteHook(WIND_TCB *pTcb)
	{
		char* name = taskName(pTcb->rtpId->initTaskId);
		
		if(strcmp(name, "FTP Server Connection Thread") == 0)
		{
			printf("FTP task deleted!\n");
			struct stat data;
			stat("constants.csv", &data);
			if(data.st_mtime != constantsLastModified)
			{
				printf("Constants file modified.  Reloading data.\n");
				CowConstants::getInstance()->restoreData();
				constantsLastModified = data.st_mtime;
			}
		}
		
		return 0;
	}
};

START_ROBOT_CLASS(CowBase);
