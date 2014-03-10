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
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_error.h"
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_module_enable.h"
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_module_enable.h"
#include "../../msg_gen/cpp/include/Modulo_GPS/msg_backup.h"
// ROS
#include "ros/ros.h"
#include "constant.h"

// Señales externas
#include "external_signals.h"

// Interaccion con usuario
#include "interaction.h"

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



