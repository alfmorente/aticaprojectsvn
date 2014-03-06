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

// Variable de continuacion de modulo
bool exitModule;

// Mensajes
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_gps.h"
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_errores.h"
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_module_enable.h"

// ROS
#include "ros/ros.h"
#include "constant.h"
#include <signal.h>


// Suscriptores
void fcn_sub_enableModule(const Modulo_GPS::msg_module_enable);

// Funciones propias

bool connect();
bool disconnect();
bool configure();
bool sendData();
bool recvData();
bool isAlive();
bool checkStateGPS();

