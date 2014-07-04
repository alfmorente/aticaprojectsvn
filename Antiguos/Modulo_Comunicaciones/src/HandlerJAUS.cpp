#include "../include/Modulo_Comunicaciones/HandlerJAUS.h"

HandlerJAUS::HandlerJAUS()
{
	eventConexion=JAUS_NO_EVENT;
}
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
				eventConexion=JAUS_EVENT_DISCONNECT;
			else if(treeEvent->getSubType()==SystemTreeEvent::SubsystemAdded)
				eventConexion=JAUS_EVENT_CONNECT;
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

