/* 
 * File:   gest_sistema.h
 * Author: carlosamores
 *
 * Created on 12 de septiembre de 2013, 12:58
 */

#ifndef GEST_SISTEMA_H
#define	GEST_SISTEMA_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* GEST_SISTEMA_H */

// Mensajes
#include "../../msg_gen/cpp/include/Modulo_Gest_Sistema/msg_errores.h"
#include "../../msg_gen/cpp/include/Modulo_Gest_Sistema/msg_habilitacion_modulo.h"
#include "../../msg_gen/cpp/include/Modulo_Gest_Sistema/msg_modo.h"

//ROS y demas librerias
#include "ros/ros.h"
#include "constant.h"
#include <iostream>

// Funciones de suscripcion
void fcn_sub_modo(const Modulo_Gest_Sistema::msg_modo);
void fcn_sub_errores(const Modulo_Gest_Sistema::msg_errores);

// Funciones propias

// Cambia el estado de todos los modulos al estado de entrada
void modeManagement(ros::NodeHandle,int);

// Espera hasta que esten listos todos los modulos con su correspondiente
// timeout por si alguno no responde, se para la aplicacion
bool waitForAllModulesReady(ros::NodeHandle);