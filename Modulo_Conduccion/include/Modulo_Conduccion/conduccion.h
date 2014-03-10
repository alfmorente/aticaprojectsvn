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
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_engine_brake.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/msg_emergency_stop.h"

//ROS
#include "ros/ros.h"
#include "constant.h"
#include <stdlib.h>

//OTROS
#include "CANCommunication.hpp"
#include "Thread.hpp"
#include "ConduccionThread.hpp"


//Atributos 
ros::NodeHandle n;        // Manejador ROS

// ----- Publicadores
ros::Publisher pub_error, pub_switch, pub_backup, pub_info_stop; 

// ----- Subscriptores
ros::Subscriber sub_navigation, sub_com_teleop, sub_engine_brake, sub_emergency_stop;  

// ----- Mensajes
Modulo_Conduccion::msg_error msg_err;
Modulo_Conduccion::msg_switch msg_switch;
Modulo_Conduccion::msg_backup msg_backup;
Modulo_Conduccion::msg_info_stop msg_info_stop;

ConduccionThread * conduccion;
CANCommunication * can;

bool finDePrograma;   // Flag que comprueba si se crea bien la comunicacion con CAN
int CANflag;          // Flag contador de reintentos de establecimiento de comunicaciones CAN
             
//Funciones de suscripcion
void fcn_sub_navigation(const Modulo_Conduccion::msg_navigation);
void fcn_sub_com_teleop(const Modulo_Conduccion::msg_com_teleop);
void fcn_sub_engine_brake(const Modulo_Conduccion::msg_engine_brake);
void fcn_sub_emergency_stop(const Modulo_Conduccion::msg_emergency_stop);

//Funciones propias
bool createCommunication();
bool disconnectCommunication();


bool sendData();
bool recvData();
bool checkConnection();
bool convertROStoCAN();
bool convertCANtoROS();