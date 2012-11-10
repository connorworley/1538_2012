//=============================================================================
// File: DashboardDataFormat.cpp
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

#include "DashboardDataFormat.h"
#include "CowRobot.h"

#include "CowConstants.h"

void sendVisionData() {
	Dashboard &dash = DriverStation::GetInstance()->GetHighPriorityDashboardPacker();
	dash.AddCluster(); // wire (2 elements)
	{
		dash.AddCluster(); // tracking data
		{
			dash.AddDouble(1.0); // Joystick X
			dash.AddDouble(135.0); // angle
			dash.AddDouble(3.0); // angular rate
			dash.AddDouble(5.0); // other X
		}
		dash.FinalizeCluster();
		dash.AddCluster(); // target Info (2 elements)
		{
			dash.AddCluster(); // targets
			{
				dash.AddDouble(100.0); // target score
				dash.AddCluster(); // Circle Description (5 elements)
				{
					dash.AddCluster(); // Position (2 elements)
					{
						dash.AddDouble(30.0); // X
						dash.AddDouble(50.0); // Y
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster(); // Position
				dash.AddDouble(45.0); // Angle
				dash.AddDouble(21.0); // Major Radius
				dash.AddDouble(15.0); // Minor Radius
				dash.AddDouble(324.0); // Raw score
			}
			dash.FinalizeCluster(); // targets
		}
		dash.FinalizeCluster(); // target Info
	}
	dash.FinalizeCluster(); // wire
	
	dash.Finalize();
}

void sendIOPortData() {
	//CowConstants * rc = CowConstants::getInstance();
	Dashboard &dash = DriverStation::GetInstance()->GetLowPriorityDashboardPacker();
	
	
	dash.AddCluster();
	{
		dash.AddFloat(CowRobot::getInstance()->getGyro()->GetAngle());
		
		dash.AddI32(CowRobot::getInstance()->getLeftEncoder()->Get());
		dash.AddFloat(CowConstants::getInstance()->getValueForKey("SensitivityHigh"));
		dash.AddFloat(CowConstants::getInstance()->getValueForKey("SensitivityLow"));
/*		dash.AddFloat(AutoModeController::getInstance()->getFilteredPitch());*/
	}
	dash.FinalizeCluster();
	//dash.Printf("SHOW ME THE MONEY!");
	dash.Finalize();
}
