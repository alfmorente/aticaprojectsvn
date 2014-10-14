
/** 
 * @file  JausHandler.cpp
 * @brief Implementación de la clase "JausHandler"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "JausHandler.h"

/** Constructor de la clase*/
JausHandler::JausHandler() {

}

/** Destructor de la clase*/
JausHandler::~JausHandler() {

}

/** 
 * Método privado que recepciona y gestiona los eventos generados en el nodo
 * JAUS NodeManager
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