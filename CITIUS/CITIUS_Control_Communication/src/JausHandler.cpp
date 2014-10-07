
/** 
 * @file  JausHandler.cpp
 * @brief Implementacion de la clase "JausHandler"
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "JausHandler.h"

/** Constructor de la clase*/
JausHandler::JausHandler() {

}

/** Destructor de la clase*/
JausHandler::~JausHandler() {

}

/** Capturador de eventos generados en NodeManager
 * @param[in] e Evento capturado
 */
void JausHandler::handleEvent(NodeManagerEvent *e) {

  switch (e->getType()) {
    case NodeManagerEvent::SystemTreeEvent:
      SystemTreeEvent *treeEvent;
      treeEvent = (SystemTreeEvent *) e;
      printf("%s\n", treeEvent->toString().c_str());
      delete e;
      break;

    case NodeManagerEvent::ErrorEvent:
      ErrorEvent *errorEvent;
      errorEvent = (ErrorEvent *) e;
      //printf("%s\n", errorEvent->toString().c_str());
      delete e;
      break;
    default:
      delete e;
      break;
  }
}