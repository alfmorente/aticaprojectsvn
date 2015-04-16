
/** 
 * @file  external_signals.h
 * @brief Colección de funciones para tratameinto de señales
 * @author Carlos Amores
 * @date 2013, 2014, 2015
 */
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>

void do_exit(int);
void signal_handler(int);
void init_signals();

