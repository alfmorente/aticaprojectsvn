/* 
 * File:   JausController.h
 * Author: atica
 *
 * Created on 9 de septiembre de 2014, 12:58
 */

#ifndef JAUSCONTROLLER_H
#define	JAUSCONTROLLER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* JAUSCONTROLLER_H */

// Librerias de JAUS
#include "jaus.h"
#include "openJaus.h"
#include "JausHandler.h"
#include <iostream>

class JausController {
public:
    
    // Patron Singleton
    static JausController *getInstance();
    
    JausController();
    
    // Inicializacion de artefactos ROS/JAUS
    void initJAUS();
    
    // Finalizacion de JAUS
    void endJAUS();

    // Generacion de mensaje de estado para Controller
    void informStatus();
    
    // Enviar mensaje
    void sendMessage(int);

    
private:
    
    // Patron Singleton
    static JausController *instance;
    static bool instanceCreated;
    
    // Manejador de JAUS 
    FileLoader *configData;
    JausHandler *handler;
    NodeManager *nm;
    
    // Controlador activo
    int subsystemController;
    int nodeController;

    // Componentes JAUS
    OjCmpt missionSpoolerComponent;
    
    // Enviar mensajes
    void sendRunMissionMessage();

};