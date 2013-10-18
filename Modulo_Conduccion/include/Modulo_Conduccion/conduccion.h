/* 
 * File:   conduccion.h
 * Author: carlosamores
 *
 * Created on 16 de septiembre de 2013, 11:32
 */

#ifndef CONDUCCION_H
#define	CONDUCCION_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* CONDUCCION_H */

//Mensajes
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_com_teleoperado.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_errores.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_gest_navegacion.h"

//ROS
#include "ros/ros.h"
#include "constant.h"
#include <stdlib.h>

//Funciones de suscripcion
void fcn_sub_com_teleop(const Modulo_Conduccion::msg_com_teleoperado);
void fcn_sub_navegacion(const Modulo_Conduccion::msg_gest_navegacion);

//Funciones propias
bool connect();
bool disconnect();
bool sendData();
bool recvData();
bool checkConnection();
bool convertROStoCAN();
bool convertCANtoROS();