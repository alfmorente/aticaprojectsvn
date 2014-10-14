/* 
 * File:   gps.h
 * Author: atica
 *
 * Created on 16 de septiembre de 2013, 11:51
 */

#ifndef GPS_H
#define	GPS_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

// Mensajes

#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_gps.h"


#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_error.h"
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_module_enable.h"
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_backup.h"

// Librerias 
#include <iostream>
//#include "TeachThread.hpp"
#include <time.h>
#include <stdlib.h> 
#include <stdio.h>
#include <math.h>

// ROS
#include "ros/ros.h"
#include "../../Common_files/include/Common_files/constant.h"
#include "constant_gps.h"
#include "GPS_Management.h"

// Señales externas
#include "external_signals.h"

// Interaccion con usuario
#include "interaction.h"


// Variable de continuacion de modulo
bool exitModule, readyToPublish;
int contFix, contSat;


// Funciones propias
bool connect();
bool disconnect();
bool configure();
bool sendData();
bool recvData();
bool isAlive();
bool checkStateGPS();
void initModuleVariables();



#endif	/* GPS_H */