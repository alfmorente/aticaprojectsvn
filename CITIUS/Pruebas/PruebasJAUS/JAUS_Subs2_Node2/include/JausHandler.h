/* 
 * File:   JausHandler.h
 * Author: Carlos Amores
 *
 * Created on 18 de junio de 2014, 12:06
 */

#include <stdlib.h>
#include "openJaus.h"


#include <cstdlib>
#include <unistd.h>
#include <termios.h>
#define CLEAR_COMMAND "clear"

#ifndef JAUSHANDLER_H
#define	JAUSHANDLER_H

#endif	/* JAUSHANDLER_H */

class JausHandler : public EventHandler
{
public:
        JausHandler();
	~JausHandler();
	void handleEvent(NodeManagerEvent *e);

};