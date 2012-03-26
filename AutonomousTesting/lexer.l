%{
#include <stdio.h>
#include "parser.h"
%}

%%
[0-9\.-]+                yylval.number=atof(yytext); return NUMBER;
[a-zA-Z0-9_][a-zA-Z0-9_]* yylval.string=strdup(yytext); return WORD;
\[                      return LBRACKET;
\]                      return RBRACKET;
;                       return SEMICOLON;
\n                      /* whitespace */;
[ \t]+                  /* whitespace */;
%%
