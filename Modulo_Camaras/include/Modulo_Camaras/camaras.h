/* 
 * File:   gps.h
 * Author: carlosamores
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
#include "../../msg_gen/cpp/include/Modulo_Camaras/msg_camaras.h"
#include "../../msg_gen/cpp/include/Modulo_Camaras/msg_errores.h"

// ROS y demas librerias
#include "ros/ros.h"
#include "constant.h"

// NO hay suscriptores

// Funciones propias

bool connect();
bool disconnect();
bool configure();
bool sendData();
bool recvData();
bool isAlive();
bool checkStateCamera();

