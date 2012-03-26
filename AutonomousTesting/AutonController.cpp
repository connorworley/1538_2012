#include "AutonController.h"

AutonController::AutonController(const char* filename)
{
	bot = RAWCRobot::getInstance();
	timer = new Timer();
	scriptFilename = filename;
	reloadCommands();
}

void AutonController::reloadCommands(const char* filename)
{
	scriptFilename = filename;
	reloadCommands();
}

void AutonController::reloadCommands()
{
	for(std::map<char*, AutonCommand*>::iterator it = commands.begin(); it != commands.end(); it++)
	{
		deleteLinkedList(it->second);
	}
	commands.clear();

	FILE* file = fopen(scriptFilename, "r");
	if(!file)
	{
		printf("Auton script file \"%s\" does not exist.  No commands loaded.\n", scriptFilename);
		return;
	}

	if(parseCommands(file) != 0)
	{
		printf("Syntax error in %s!\n", scriptFilename);
		for(std::map<char*, AutonCommand*>::iterator it = commands.begin(); it != commands.end(); it++)
		{
			deleteLinkedList(it->second);
		}
		commands.clear();
	}
	
	selector = commands.begin();
	
	timer->Start();

	fclose(file);
}

void AutonController::handle()
{
	AutonCommand* curCmd = selector->second;
	if(curCmd == NULL)
		return;
	
	bool result = false;
	
	if(curCmd->type == "DRIVE")
	{
		
	} else if(curCmd->type == "TURN")
	{
		
	} else if(curCmd->type == "SHOOTER")
	{
		bot->getShooter()->SetSpeed(curCmd->parameter);
		result = true;
	} else if(curCmd->type == "ARMS")
	{
		if(curCmd->parameter == 1)
			bot->getFunnel()->Set(true);
		else
			bot->getFunnel()->Set(false);
	} else if(curCmd->type == "INTAKE")
	{
		if(curCmd->parameter == 1)
			bot->getIntake()->Set(Relay::kForward);
		else if(curCmd->parameter == -1)
			bot->getIntake()->Set(Relay::kReverse);
		else
			bot->getIntake()->Set(Relay::kOff);
		
		result = true;
	} else if(curCmd->type == "CHUTE")
	{
		if(curCmd->parameter == 1)
			bot->getChute()->Set(Relay::kForward);
		else if(curCmd->parameter == -1)
			bot->getChute()->Set(Relay::kReverse);
		else
			bot->getChute()->Set(Relay::kOff);
		
		result = true;
	} else if(curCmd->type == "WAIT_FOR_BALLS")
	{
		result = bot->getBallCount() >= curCmd->parameter;
	} else if(curCmd->type == "NOTHING")
	{
		// waiting...
	} else
	{
		printf("Unknown command \"%s\" in script %s (file %s)!\n", curCmd->type, selector->first, scriptFilename);
	}
	
	if(result || timer->Get() > curCmd->timeout)
	{
		selector->second = curCmd->next;
		delete curCmd;
		timer->Reset();
	}
}

void AutonController::addCommand(char* scriptName, char* type, double param, double timeout)
{
	printf("Adding %s (%f) to %s with timeout of %f\n", type, param, scriptName, timeout);

	AutonCommand* c = new AutonCommand;
	c->type = type;
	c->parameter = param;
	c->timeout = timeout;
	c->next = NULL;

	// insert into the linked list
	AutonCommand** x = &commands[scriptName];
	while(*x != NULL)
	{
		x = &((**x).next);
	}
	*x = c;
}

void AutonController::cycleMode()
{
	selector++;
	if(selector == commands.end())
		selector = commands.begin();
}

char* AutonController::getModeName()
{
	return selector->first;
}

void AutonController::deleteLinkedList(AutonCommand* item)
{
	AutonCommand* n = item->next;
	delete item;
	if(n != NULL)
		return deleteLinkedList(n);
}
