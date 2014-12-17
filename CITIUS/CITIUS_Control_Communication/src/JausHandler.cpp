
/** 
 * @file  JausHandler.cpp
 * @brief Implementación de la clase "JausHandler"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "JausHandler.h"
#include "constant.h"

/** Constructor de la clase*/
JausHandler::JausHandler() {
  mycAvailable = false;
  tabletAvailable = false;

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
      // Actualizacion subsistema MyC añadido
      if(treeEvent->getSubType() == SystemTreeEvent::SubsystemAdded && treeEvent->getSubsystem()->id == JAUS_SUBSYSTEM_MYC){
        mycAvailable = true;
      }
      // Actualizacion subsistema MyC eliminado
      /*if((treeEvent->getSubType() == SystemTreeEvent::SubsystemRemoved || treeEvent->getSubType() == SystemTreeEvent::SubsystemTimeout) && treeEvent->getSubsystem()->id == JAUS_SUBSYSTEM_MYC){
        mycAvailable = false;
      }*/
      // Actualizacion nodo tablet añadido
      if(treeEvent->getSubType() == SystemTreeEvent::NodeAdded && treeEvent->getNode()->id == JAUS_NODE_TABLET){
        tabletAvailable = true;
      }
      // Actualizacion nodo tablet eliminado
      /*if((treeEvent->getSubType() == SystemTreeEvent::NodeRemoved || treeEvent->getSubType() == SystemTreeEvent::NodeTimeout) && treeEvent->getSubsystem()->id == JAUS_SUBSYSTEM_UGV && treeEvent->getNode()->id == JAUS_NODE_TABLET){
        tabletAvailable = false;
      }*/
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

/**
 * Método público que indica si en el momento de la consulta está disponible el
 * puesto de mando y control en la arquitectura JAUS
 * @return Booleano que indica la disponibilidad del subsistema
 */
bool JausHandler::isMyCAvailable(){
  return mycAvailable;
}

/**
 * Método público que indica si en el momento de la consulta está disponibe el 
 * tablet en la arquitectura JAUS
 * @return Booleano que indica la disponibilidad del subsistema
 */
bool JausHandler::isTabletAvailable(){
  return tabletAvailable;
}
