/* 
 * File:   simuladorDriving.h
 * Author: atica
 *
 * Created on 13 de marzo de 2014, 10:23
 */

#ifndef SIMULADORDRIVING_H
#define	SIMULADORDRIVING_H

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <stdlib.h>

// Cabeceras de mensajes de publicacion
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_error.h"
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_switch.h"
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_backup.h"
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_info_stop.h"

// Cabeceras de mensajes de subscripcion
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_navigation.h"
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_com_teleop.h"
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_fcn_aux.h"
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_emergency_stop.h"

#include "../../Common_files/include/Common_files/constant.h"


// ----- Publicadores
ros::Publisher pub_com_teleop, pub_fcn_aux, pub_emergency_stop; 

// ----- Subscriptores
ros::Subscriber sub_error, sub_switch, sub_backup, sub_info_stop, sub_emergency_stop;

// ----- Mensajes
Common_files::msg_com_teleop msg_com_teleop;
Common_files::msg_fcn_aux msg_fcn_aux;
Common_files::msg_emergency_stop msg_emergency_stop;


// Variables
int acelerador;
int velocidad;
int freno;
int direccion;
int marcha;
int freno_mano;
int motor;
int lucesIR;
int luces;
int diferenciales;
int activacionLaser;
int cont_emergency;
int cont_engine_brake;

int cont;

// funciones propias del simulador
void inicializaVariables();
void publicaComTeleop();
void publicaEmergencyStop();
void publicaFcnAux();

//Funciones de suscripcion
void fcn_sub_error (const Common_files::msg_error);
void fcn_switch (const Common_files::msg_switch);
void fcn_backup (const Common_files::msg_backup);
void fcn_sub_info_stop (const Common_files::msg_info_stop);
void fcn_sub_emergency_stop (const Common_files::msg_emergency_stop);


#endif	/* SIMULADORDRIVING_H */

