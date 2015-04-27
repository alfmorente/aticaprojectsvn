/**
 * @file   HandlerJAUS.cpp
 * @brief  Fichero fuente para el manejador de JAUS
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */
#include "../include/Modulo_Comunicaciones/HandlerJAUS.h"

/**
 * Constructor de la clase
 */
HandlerJAUS::HandlerJAUS()
{
	eventConexion=JAUS_NO_EVENT;
}

/**
 * Método que recibe los eventos producidos en la comunicación JAUS
 * @param[io] e Evento recibido por otro nodo o subsistema
 */
void HandlerJAUS::handleEvent(NodeManagerEvent *e)
{
	SystemTreeEvent *treeEvent;
	ErrorEvent *errorEvent;
	JausMessageEvent *messageEvent;
	DebugEvent *debugEvent;
	ConfigurationEvent *configEvent;
	string evento;

	JausMessage jausMessage;
	switch(e->getType())
	{
		case NodeManagerEvent::SystemTreeEvent:
			treeEvent = (SystemTreeEvent *)e;
			evento=treeEvent->toString();
			if(treeEvent->getSubType()==SystemTreeEvent::SubsystemRemoved)
                        {                           
                            if(treeEvent->getSubsystem()->id==1)
                            {
				eventConexion=JAUS_EVENT_DISCONNECT;
                                printf("Desconectado Subsistema %d\n",treeEvent->getSubsystem()->id);                        
                            }
                        }
			else if(treeEvent->getSubType()==SystemTreeEvent::SubsystemAdded)
                        {
                            printf("Conectado Subsistema %d\n",treeEvent->getSubsystem()->id);                            
                            if(treeEvent->getSubsystem()->id==1)
                                   eventConexion=JAUS_EVENT_CONNECT;
                        }
			/**else if(treeEvent->getSubType()==SystemTreeEvent::SubsystemTimeout)
                        {
                            if(treeEvent->getSubsystem()->id==1)
                                   eventConexion=JAUS_EVENT_LOSED;
                        }**/                        

			printf("%s\n", treeEvent->toString().c_str());
			delete e;
			break;

		case NodeManagerEvent::ErrorEvent:
			errorEvent = (ErrorEvent *)e;
			//printf("%s\n", errorEvent->toString().c_str());
			delete e;
			break;

		case NodeManagerEvent::JausMessageEvent:
			messageEvent = (JausMessageEvent *)e;
			jausMessage=messageEvent->getJausMessage();
			delete e;
			break;

		case NodeManagerEvent::DebugEvent:
			debugEvent = (DebugEvent *)e;
			//printf("%s\n", debugEvent->toString().c_str());
			delete e;
			break;

		case NodeManagerEvent::ConfigurationEvent:
			configEvent = (ConfigurationEvent *)e;
			//printf("%s\n", configEvent->toString().c_str());
			delete e;
			break;

		default:
			delete e;
			break;
	}
}

/**
 * Método que controla los eventos de conexión o desconexión de JAUS
 * @return Entero indicando el tipo de evento
 */
int HandlerJAUS::controlJaus()
{
	if(eventConexion==JAUS_EVENT_DISCONNECT)
	{
		eventConexion=JAUS_NO_EVENT;
		return JAUS_EVENT_DISCONNECT;
	}
	else if(eventConexion==JAUS_EVENT_CONNECT)
	{
		eventConexion=JAUS_NO_EVENT;
		return JAUS_EVENT_CONNECT;
	}
	else
		return JAUS_NO_EVENT;

}

