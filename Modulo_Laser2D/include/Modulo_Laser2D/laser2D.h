/* 
 * File:   laser2D.h
 * Author: carlosamores
 *
 * Created on 16 de septiembre de 2013, 11:45
 */

#ifndef LASER2D_H
#define	LASER2D_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* LASER2D_H */

// Mensajes
#include "../../msg_gen/cpp/include/Modulo_Laser2D/msg_laser.h"
#include "../../msg_gen/cpp/include/Modulo_Laser2D/msg_errores.h"

// ROS
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
bool checkStateLaser();

