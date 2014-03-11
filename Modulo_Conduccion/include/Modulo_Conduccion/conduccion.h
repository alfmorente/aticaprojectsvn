/* 
 * File:   conduccion.h
 * Author: Sergio Doctor López
 *
 * Created on 5 de marzo de 2014, 10:50
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

// Cabeceras de mensajes de publicacion
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_error.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_switch.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_backup.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_info_stop.h"

// Cabeceras de mensajes de subscripcion
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_navigation.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_com_teleop.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_fcn_aux.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_emergency_stop.h"

//ROS
#include "ros/ros.h"
#include "constant.h"
#include <stdlib.h>

//OTROS
#include "CANCommunication.hpp"
#include "Thread.hpp"
#include "ConduccionThread.hpp"

// Interaccion con usuario
#include "interaction.h"

//Atributos 

// Para interraccionar con los usuarios
int operationMode;

// ----- Publicadores
ros::Publisher pub_error, pub_switch, pub_backup, pub_info_stop, pub_emergency_stop; 

// ----- Subscriptores
ros::Subscriber sub_navigation, sub_com_teleop, sub_fcn_aux, sub_emergency_stop;  

// ----- Mensajes
Modulo_Conduccion::msg_error msg_err;
Modulo_Conduccion::msg_switch msg_switch;
Modulo_Conduccion::msg_backup msg_backup;
Modulo_Conduccion::msg_info_stop msg_info_stop;
Modulo_Conduccion::msg_emergency_stop msg_emergency_stop;

ConduccionThread * conduccion;
CANCommunication * can;

bool finDePrograma;   // Flag que comprueba si se crea bien la comunicacion con CAN
int CANflag;          // Flag contador de reintentos de establecimiento de comunicaciones CAN
             
//Funciones de suscripcion
void fcn_sub_navigation(const Modulo_Conduccion::msg_navigation);
void fcn_sub_com_teleop(const Modulo_Conduccion::msg_com_teleop);
void fcn_sub_engine_brake(const Modulo_Conduccion::msg_fcn_aux);
void fcn_sub_emergency_stop(const Modulo_Conduccion::msg_emergency_stop);

//Funciones propias
bool createCommunication();
bool disconnectCommunication();

// Funciones tratamiento de señales
void do_exit(int);              // what has to be done at program exit
void signal_handler(int);       // the signal handler for manual break Ctrl-C
void init_signals();            // what has to be done at program start


bool sendData();
bool recvData();
bool checkConnection();
bool convertROStoCAN();
bool convertCANtoROS();