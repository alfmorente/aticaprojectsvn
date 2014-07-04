/* 
 * File:   teleoperacion.h
 * Author: atica
 * 
 * Created on 13 de septiembre de 2013, 11:27
 */

#ifndef TELEOPERACION_H
#define	TELEOPERACION_H

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* TELEOPERACION_H */

//Mensajes
#include "Common_files/msg_com_teleop.h"
#include "Common_files/msg_error.h"
#include "Common_files/msg_module_enable.h"
#include "Common_files/msg_mode.h"
#include "Common_files/srv_data.h"

//ROS y demás librerias
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "../../../src/Common_files/include/Common_files/constant.h"
#include <stdlib.h>
#include <signal.h>

// Publicadores y suscriptores
ros::Publisher pub_error;
ros::Publisher pub_teleop;
ros::Subscriber sub_hab_modulo;
ros::Subscriber sub_com_teleop;
ros::ServiceServer server;


//Variables globales
bool enableModule;
int error_count;
bool end_error;
//Variable de activacion de modulo
bool exitModule;

// Obtener el modo de operacion
int getOperationMode(int,char **);
// Sintaxis correcta ante fallo
void printCorrectSyntax();

//Funciones de suscripcion
void fcn_sub_com_teleop(const Common_files::msg_com_teleopPtr&);
void fcn_sub_enable_module(const Common_files::msg_module_enablePtr&);
bool fcn_heartbeat(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp);

//Funciones propias
void initialize(ros::NodeHandle n);
int convertToCorrectValues(int,int);

#define MAX_OUTRANGE_ERROR 3
// Valores máximos y mínimos de los comandos de teleoperación
#define MIN_STEER_VALUE -100
#define MAX_STEER_VALUE 100
#define MIN_THROTTLE_VALUE 0
#define MAX_THROTTLE_VALUE 100
#define MIN_BRAKE_VALUE 0
#define MAX_BRAKE_VALUE 100
#define MIN_HANDBRAKE_VALUE 0
#define MAX_HANDBRAKE_VALUE 1
#define MIN_GEAR_VALUE 0
#define MAX_GEAR_VALUE 4
#define MIN_LIGHT_VALUE 0
#define MAX_LIGHT_VALUE 1
#define MIN_LIGHT_IR_VALUE 0
#define MAX_LIGHT_IR_VALUE 1
#define MIN_ENGINE_VALUE 0
#define MAX_ENGINE_VALUE 1
#define MIN_DIFF_VALUE 0
#define MAX_DIFF_VALUE 1
#define MIN_LASER_VALUE 0
#define MAX_LASER_VALUE 1
