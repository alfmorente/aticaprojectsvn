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

// Defines propios del módulo

// Timeout para activacion de modulos
#define TIMEOUT_ACTIVATION_MODULE 10 //segundos
#define TIMEOUT_ACK 0.9
#define NUM_MODULES 13

// Tipo de ack
#define CONVOY_ACK 0
#define EMERGENCY_ACK 1

// Modo convoy follower
#define LEADER 0
#define FOLLOWER 1

// Mensajes
#include "Common_files/msg_error.h"
#include "Common_files/msg_module_enable.h"
#include "Common_files/msg_mode.h"
#include "Common_files/msg_available.h"
#include "Common_files/msg_emergency_stop.h"
#include "Common_files/msg_switch.h"
#include "Common_files/msg_fcn_aux.h"
#include "Common_files/srv_data.h"



//ROS y demas librerias
#include "ros/ros.h"
#include <iostream>

//Variables
int state_remote;
int state_navigation;
int state_driving;
int state_gps;
int state_laser3D;
int state_frontLaser1;
int state_frontLaser2;
int state_humanLocalization;
int state_beacon;
int state_waypointsMap;
int state_rangeDataFusion;
int state_errorMgmnt;
int state_camera;
int state_convoy;
int state_communication;

int actualMode;
bool emergencyACK;
bool convoyACK;
bool carType;
bool stoppingCar;


//Publicadores
ros::Publisher pub_module_enable;
ros::Publisher pub_error;
ros::Publisher pub_mode_error;
ros::Publisher pub_mode_communication;
ros::Publisher pub_mode_convoy;
ros::Publisher pub_fcn_aux;
ros::Publisher pub_emergency_stop;
ros::Publisher pub_fcn_aux_ack;

//Servidor de Parametros
ros::ServiceServer server;

bool modesAvailables[13];

// Funciones de suscripcion
void fcn_sub_mode_error(const Common_files::msg_mode);
void fcn_sub_mode_communication(const Common_files::msg_mode);
void fcn_sub_mode_convoy(const Common_files::msg_mode);
void fcn_sub_fcn_aux(const Common_files::msg_fcn_aux);
void fcn_sub_switch(const Common_files::msg_switch);
void fcn_sub_available(const Common_files::msg_available);
void fcn_sub_emergency_stop(const Common_files::msg_emergency_stop);

bool fcn_server_data(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp);



// Funciones propias

// Cambia el estado de todos los modulos al estado de entrada
void modeManagement(ros::NodeHandle,int);

// Espera hasta que esten listos todos los modulos con su correspondiente
// timeout por si alguno no responde, se para la aplicacion
bool waitForAllModulesReady(ros::NodeHandle);

//Actualiza el estado de los modulos
void updateStatusModules(ros::NodeHandle);

//Metodos auxiliares
bool modeRUN(int mode);
void modeSTOP(int mode);
bool modeEXIT(int mode);
void modeRESUME(int mode);
bool emergencySTOP();
bool timerACK(double,int);
