/* 
 * File:   external_signals.h
 * Author: atica
 *
 * Created on 8 de octubre de 2014, 10:58
 */
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

// Funciones tratamiento de señales
// what has to be done at program exit
void do_exit(int);

// the signal handler for manual break Ctrl-C
void signal_handler(int);

// what has to be done at program start
void init_signals();

