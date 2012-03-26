#ifndef _AUTON_CONTROLLER_H
#define _AUTON_CONTROLLER_H
#include "../RAWCRobot.h"
#include "../RAWCLib.h"

using namespace RAWCLib;

typedef struct AutonCommand
{
	char* type;
	double parameter;
	double timeout;
	AutonCommand* next;
} AutonCommand;

class AutonController
{
public:
	AutonController(const char* filename);
	
	void reloadCommands();
	void reloadCommands(const char* filename);
	
	void handle();
	
	void addCommand(char* scriptName, char* type, double param, double timeout);
	
private:
	RAWCRobot* bot;
	
	const char* scriptFilename;
	map<char*, AutonCommand*> commands;
	
	map<char*, AutonCommand*>::iterator selector;
	void cycleMode();
	char* getModeName();
	
	Timer* timer;
	
	int parseCommands(FILE* file);
	void deleteLinkedList(AutonCommand* item);
};

#endif
