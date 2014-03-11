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
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_gps.h"
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_error.h"
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_module_enable.h"
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_stream.h"
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_backup.h"

// Librerias basicas
#include <iostream>

// ROS
#include "ros/ros.h"
#include "constant.h"

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
bool exitModule, readyToPublish;
// Variable donde guardar datos de INS
insData insdata;

// Suscriptores
void fcn_sub_enableModule(const Modulo_GPS::msg_module_enable);
void fcn_sub_backup(const Modulo_GPS::msg_backup);

// Funciones propias

bool connect();
bool disconnect();
bool configure();
bool sendData();
bool recvData();
bool isAlive();
bool checkStateGPS();
void initModuleVariables();



