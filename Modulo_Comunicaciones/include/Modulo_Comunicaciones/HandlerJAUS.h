/* 
 * File:   handlerJAUS.h
 * Author: atica
 *
 * Created on 19 de noviembre de 2013, 12:51
 */

#ifndef HANDLERJAUS_H
#define	HANDLERJAUS_H
#if defined(WIN32)
	#include <windows.h>
	#define CLEAR_COMMAND "cls"
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <cstdlib>
	#include <unistd.h>
	#include <termios.h>
	#define CLEAR_COMMAND "clear"
#endif

#include <jaus.h>      // Header file for JAUS types, structs and messages
#include <openJaus.h>  // Header file for the OpenJAUS specific C/C++ code base

#define JAUS_NO_EVENT 0
#define JAUS_EVENT_CONNECT 1
#define JAUS_EVENT_DISCONNECT 2

class HandlerJAUS : public EventHandler
{

	public:
		volatile int eventConexion;
	public:
		HandlerJAUS();
	        void handleEvent(NodeManagerEvent *e);
		int  controlJaus();
};




#endif	/* HANDLERJAUS_H */

