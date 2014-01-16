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
#include "../../msg_gen/cpp/include/Modulo_Gest_Sistema/msg_error.h"
#include "../../msg_gen/cpp/include/Modulo_Gest_Sistema/msg_module_enable.h"
#include "../../msg_gen/cpp/include/Modulo_Gest_Sistema/msg_mode.h"

//ROS y demas librerias
#include "ros/ros.h"
#include "constant.h"
#include <iostream>

//Variables
int state_teleop;
int state_nav;
int state_drive;
int state_gps;
int state_laser3D;
int state_laserBackRight;
int state_laserBackLeft;
int state_laserFront;
int state_humanLocalization;
int state_beacon;
//int state_waypointsMap;
int state_rangeDataFusion;
int state_errors;
int actualMode;

// Funciones de suscripcion
void fcn_sub_modo(const Modulo_Gest_Sistema::msg_mode);
void fcn_sub_errores(const Modulo_Gest_Sistema::msg_error);



// Funciones propias

// Cambia el estado de todos los modulos al estado de entrada
void modeManagement(ros::NodeHandle,int);

// Espera hasta que esten listos todos los modulos con su correspondiente
// timeout por si alguno no responde, se para la aplicacion
bool waitForAllModulesReady(ros::NodeHandle);

//Actualiza el estado de los modulos
void updateStatusModules(ros::NodeHandle);