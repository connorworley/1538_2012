#include "AutonController.h"

AutonController::AutonController(const char* filename)
{
	//timer = new Timer();
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
		deleteLinkedList((*it).second);
	}
	commands.clear();
	//timer->Reset();

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
			deleteLinkedList((*it).second);
		}
		commands.clear();
	}

	fclose(file);
}

void AutonController::handle()
{

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

void AutonController::deleteLinkedList(AutonCommand* item)
{
	AutonCommand* n = item->next;
	delete item;
	if(n != NULL)
		return deleteLinkedList(n);
}
