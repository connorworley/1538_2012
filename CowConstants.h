#ifndef _Cow_CONSTANTS_H
#define _Cow_CONSTANTS_H
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
#include <string>
#include <map>

#define Cow_CONSTANTS_DEFAULT_FILE		"constants.csv"

class CowConstants
{
	// A CowConstant is contained in CowConstants
	typedef double CowConstant;
	static const CowConstant Cow_CONSTANTS_DEFAULT_RET_VAL = 0.0;
	
private:
	map<string, CowConstant> data;
	static CowConstants * singletonInstance;
	
	CowConstants();
	//TODO: Add functionality to let this map persist to a file	

	void restoreDataFromFile(string fileName);
	
	
public:
	void restoreData(); // from default file
	void save(); // Save to the default file
	void saveDataToFile(string fileName); // Save to another file

	// Get the shared object
	static CowConstants * getInstance();
	
	// Main mechanism to input data
	void insertKeyAndValue(string key, CowConstant value);
	
	// Main mechanism to look up data
	// NOTE: I need to figure out how this can fail elegantly 
	// 		instead of just checking to see if it exists first
	//		Maybe pass in a pointer to a CowConstant and write to that?
	CowConstant getValueForKey(string key);
	bool doesKeyExist(string key);
	
};

#endif // _Cow_CONSTANTS_H
