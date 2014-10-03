
/** 
 * @file  JausHandler.h
 * @brief Declara el tipo de la clase "JausHandler"
 * - La clase implementa el manejo del NodeManager en la arquitectura JAUS
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef JAUSHANDLER_H
#define	JAUSHANDLER_H

#include <stdlib.h>
#include "openJaus.h"
#include <cstdlib>
#include <unistd.h>
#include <termios.h>

#endif	/* JAUSHANDLER_H */

class JausHandler : public EventHandler {
public:
  JausHandler();
  ~JausHandler();
private:

  // Manejador de eventos
  void handleEvent(NodeManagerEvent *e);
};