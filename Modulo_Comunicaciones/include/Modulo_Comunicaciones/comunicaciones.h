/* 
 * File:   comunicaciones.h
 * Author: carlosamores
 *
 * Created on 12 de septiembre de 2013, 12:48
 */

#ifndef COMUNICACIONES_H
#define	COMUNICACIONES_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* COMUNICACIONES_H */

// Mensajes
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_camaras.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_com_teleoperado.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_errores.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_gps.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_modo.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_waypoint.h"

//ROS y demás librerias
#include "ros/ros.h"
#include "constant.h"
#include "../../libjaus/include/jaus.h"
#include "../../libopenJaus/include/openJaus.h"

// Estructura de datos que maneja los mensajes ROS
typedef struct{
    short tipo_mensaje;
    Modulo_Comunicaciones::msg_camaras mens_cam;
    Modulo_Comunicaciones::msg_gps mens_gps;
    Modulo_Comunicaciones::msg_errores mens_errores;
    Modulo_Comunicaciones::msg_modo mens_modo;
    Modulo_Comunicaciones::msg_waypoint mens_waypoint;
    Modulo_Comunicaciones::msg_com_teleoperado mens_teleop;
    //Modulo_Comunicaciones::msg_backup mens_backup;
}ROSmessage;

// Estructura de datos que maneja los mensajes JAUS
typedef struct{
   // short tipo_mensaje;
    ReportImageMessage imagen;
    ReportGlobalPoseMessage gps;
    ReportAlarmaMessage alarma;
}mensajeJAUS;

// Funciones de suscripcion
void fcn_sub_gps(const Modulo_Comunicaciones::msg_gps);
void fcn_sub_errores(const Modulo_Comunicaciones::msg_errores);
void fcn_sub_camaras(const Modulo_Comunicaciones::msg_camaras);

// Funciones propias
bool connect();
bool disconnect();
bool checkConnection();
JausMessage convertROStoJAUS(ROSmessage msg_ROS);
ROSmessage convertJAUStoROS(mensajeJAUS msg_JAUS);
bool sendJAUSMessage(JausMessage);
bool rcvJAUSMessage();