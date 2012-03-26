%{
#include <stdio.h>
#include <string.h>
#include "AutonController.h"
 
void yyerror(const char *str)
{
        //printf("error: %s\n",str);
}
 

extern "C"
{
        int yyparse(void);
        int yylex(void);  
        int yywrap()
        {
                return 1;
        }
}

extern FILE* yyin;

AutonController* controller = NULL;

int AutonController::parseCommands(FILE* file)
{
	controller = this;
	yyin = file;
	return yyparse();
} 

char* section = NULL;

%}

%token <number> NUMBER
%token <string> WORD
%token <string> LBRACKET
%token <string> RBRACKET
%token <string> SEMICOLON

%union 
{
        double number;
        char* string;
}

%%
start:
	section commands
	;	

commands:
	|
	commands command
	;

command:
	section
	|
	WORD NUMBER NUMBER SEMICOLON
	{
		controller->addCommand(section, $1, $2, $3);
	}
	;
	
section:
	LBRACKET WORD RBRACKET
	{
		section = $2;
	}
	;
%%
