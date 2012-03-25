#!/bin/bash
lex lexer.l
yacc -o parser.cc -d parser.y