//=============================================================================
// File: CowLib.h
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

#ifndef CowLib_H_
#define CowLib_H_
#include "WPILib.h"
#include "CowRobot.h"
#include "Declarations.h"
#include "Timer.h"
#include "DashboardDataFormat.h"
#include <algorithm>

namespace CowLib
{
	float VictorLinearize(double goal_speed);
	float LimitMix(float value, float maxValue);
	float LimitMix(float value);
	float AnalogInScale(float oldx, double center);
	
	inline bool compareParticleAnalysisReport(const ParticleAnalysisReport& a, const ParticleAnalysisReport& b)
	{
		//Want the taller one as the last element
		return a.center_mass_y_normalized > b.center_mass_y_normalized;
	}
	
	template <class IT>
	void QSortParticleAnalysisReport(IT begin, IT end) 
	{
	  std::sort(begin, end, compareParticleAnalysisReport); // ;)
	}  
	
	class PrintToLCD
	{
	public:
		static void print(bool enabled, int lineNumber, int startChar, const char * text);
		static void print(bool enabled, int lineNumber, int startChar, const char * text, float var1);
		static void print(bool enabled, int lineNumber, int startChar, const char * text, float var1, float var2);
		static void finalizeUpdate();
	};
}


#endif
