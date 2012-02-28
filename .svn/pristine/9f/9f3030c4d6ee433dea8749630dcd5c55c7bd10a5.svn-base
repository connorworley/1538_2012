/* File Name: RAWCLib.h
 * Function: header for our extra helper functions/objects
 * 
 * Author: Jakub Fiedorowicz (poland2005@gmail.com)
 * Student Lead: PJ Goesseringer (greeneyehawk@gmail.com)
 * 
 * SVN Repository: https://free2.projectlocker.com/Team254Robotics/2010Code/svn
 * 
 * This code is confidential and cannot be released or published
 * without explicit permission in writing from the Author.
 */
#ifndef RAWCLib_H_
#define RAWCLib_H_
#include "WPILib.h"
#include "RAWCRobot.h"
#include "Declarations.h"
#include "Timer.h"
#include "DashboardDataFormat.h"
#include <algorithm>

namespace RAWCLib
{
	float VictorLinearize(double goal_speed);
	float LimitMix(float value);
	float AnalogInScale(float oldx, double center);
	
	template <class IT>
	void quicksort(IT begin, IT end) {
	  std::sort(begin, end); // ;)

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
