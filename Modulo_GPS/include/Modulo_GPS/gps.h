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

#endif	/* GPS_H */

// Mensajes
#include "Common_files/msg_gps.h"
#include "Common_files/msg_error.h"
#include "Common_files/msg_module_enable.h"
#include "Common_files/msg_stream.h"
#include "Common_files/msg_backup.h"

// Librerias 
#include <iostream>
#include "TeachThread.hpp"

// ROS
#include "ros/ros.h"
#include "../../Common_files/include/Common_files/constant.h"

// Señales externas
#include "external_signals.h"

// Interaccion con usuario
#include "interaction.h"

// Estructura para almacenar los datos del gps
typedef struct{
    float latitude;
    float longitude;
    float altitude;
    float roll;
    float pitch;
    float yaw;
}insData;

// Variable de continuacion de modulo
bool exitModule, readyToPublish, teachActive, launchTeach;
// Variable donde guardar datos de INS
insData insdata;
// Hilo que controla el teach
TeachThread teachThread;
// Estructura para cola de datos en el hilo de teach
TeachData teachData;

// Suscriptores
void fcn_sub_enableModule(const Common_files::msg_module_enable);
void fcn_sub_backup(const Common_files::msg_backup);

// Funciones propias

bool connect();
bool disconnect();
bool configure();
bool sendData();
bool recvData();
bool isAlive();
bool checkStateGPS();
void initModuleVariables();



