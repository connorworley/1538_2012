//=============================================================================
// File: CowConstants.cpp
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

#include "CowConstants.h"
#include "WPILib.h"
#include "nivision.h"
#include <iostream>
#include <fstream>
#include <sstream>

// Private NI function needed to write to the VxWorks target 
IMAQ_FUNC int Priv_SetWriteFileAllowed(UINT32 enable); 

// Singleton instance
CowConstants *  CowConstants::singletonInstance = NULL;

// Helper function for converting a string to a float
template <class T>
bool from_string(T& t, 
                 const std::string& s, 
                 std::ios_base& (*f)(std::ios_base&))
{
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}


/// This static method returns a shared instance of CowConstants
/// @return The shared instance of CowConstants
CowConstants * CowConstants::getInstance()
{
	if(singletonInstance == NULL)
	{
		singletonInstance = new CowConstants();
		// We need this to write to files
		Priv_SetWriteFileAllowed(1);
	}
	return singletonInstance;
}

/// Constructor
CowConstants::CowConstants()
{
	restoreData();
}

/// Returns a bool indicating if the key/value pair exists 
bool CowConstants::doesKeyExist(string key)
{
	return (bool) data.count(key);
}

/// Returns the stored value for a string key
/// @param key
/// @return Stored Value
CowConstants::CowConstant CowConstants::getValueForKey(string key)
{
	if( doesKeyExist(key) )
	{
		return data[key];
	}
	else 
		return Cow_CONSTANTS_DEFAULT_RET_VAL;
}

/// Insert a value into the constant storage system
/// @param key String key
/// @param value CowConstant value
void CowConstants::insertKeyAndValue(string key, CowConstant value)
{
	data[key] = value;
}

/// Save data to the default file
void CowConstants::save()
{
	saveDataToFile(Cow_CONSTANTS_DEFAULT_FILE);
}

/// Save data to a file
/// @param fileName File to save to
void CowConstants::saveDataToFile(string fileName)
{
	Priv_SetWriteFileAllowed(1);
	map<string, CowConstant>::iterator it; //Iterator for map
	
	//Open a file
	ofstream outfile;
	outfile.open (fileName.c_str());
	
	//if it fails, return
	// print content:
	for ( it = data.begin() ; it != data.end(); it++ ){
		printf("Writing: %s - %f\r\n", (*it).first.c_str(),(*it).second);
		outfile << (*it).first << ", " << (*it).second << "\r\n";
	}
	
	outfile.close();
}

/// Restores constant system data from the default file
void CowConstants::restoreData()
{
	restoreDataFromFile(Cow_CONSTANTS_DEFAULT_FILE);
}

/// Restores constant system data from a file
/// @param fileName File to read from
void CowConstants::restoreDataFromFile(string fileName)
{
	string key;
	string valueString;
	
	ifstream infile ( Cow_CONSTANTS_DEFAULT_FILE );
	//printf("In restore\r\n ");
	if (infile.is_open())
	{	
		//printf("File is open\r\n");
		while (! infile.eof() )
		{
			//Get the key name
			getline (infile, key, ',');
			// get the value string
			getline(infile, valueString);
			
			// Handle weird new line situations
			if(infile.eof())
				break;
			
			// Convert value string to a number
			CowConstant value;
			if(from_string<CowConstant>(value, valueString, std::dec))
			{
				printf("%s: %f\n", key.c_str(), value);
				data[key] = value;
			}
			else
			{
				//printf( "from_string failed\r\n");
			}
	    }
		infile.close();
	}

	else printf("Unable to open file"); 	
}
