/**
 * @file   HandlerJAUS.h
 * @brief  Fichero de cabecera para el manejador de JAUS
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 * @addtogroup CommVehicle
 * @{
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

#define JAUS_NO_EVENT 0 ///< Constante que indica que no hay evento JAUS
#define JAUS_EVENT_CONNECT 1 ///< Constante que indica que se ha conectado otro subsistema JAUS
#define JAUS_EVENT_DISCONNECT 2 ///< Constante que indica que se ha desconectado otro subsistema JAUS
//#define JAUS_EVENT_LOSED 3 ///< Constante que indica comunicación perdida 


/**
 * \class HandlerJAUS
 * \brief Clase que maneja JAUS
 */
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

/**
 *@}
 */