%{
#include <stdio.h>
#include <string.h>
 
void yyerror(const char *str)
{
        fprintf(stderr,"error: %s\n",str);
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

int parse_auton(const char* fileName)
{
	FILE *f = fopen(fileName, "r");
	if(!f) {
		printf("failure! can't read %s\n", fileName);
		return -1;
	}

	yyin = f;
	do {
        	yyparse();
	} while(!feof(yyin));
} 

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
commands:
	|
	commands command
	;

command:
	LBRACKET WORD RBRACKET
	{
		printf("In section: %s\n",$2);
	}
	|
	WORD NUMBER NUMBER SEMICOLON
	{
		printf("%s %f (timeout %f)\n",$1,$2,$3);
	}
	;
%%
