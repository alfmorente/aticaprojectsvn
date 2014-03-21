/* 
 * File:   conduccion.h
 * Author: Sergio Doctor López
 *
 * Created on 10 de marzo de 2014, 10:50
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
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_error.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_switch.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_backup.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_info_stop.h"

// Cabeceras de mensajes de subscripcion
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_navigation.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_com_teleop.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_fcn_aux.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_emergency_stop.h"

//ROS
#include "ros/ros.h"

//OTROS
#include "../../../Common_files/include/Common_files/constant.h"
#include <stdlib.h>
#include "CANCommunication.hpp"
#include "Thread.hpp"
#include "ConduccionThread.hpp"

// Interaccion con usuario
#include "interaction.h"



/*******************************************************************************
 *******************************************************************************
 *                              ATRIBUTOS
 * *****************************************************************************
 * ****************************************************************************/

// Para interraccionar con los usuarios
int operationMode;

// ----- Publicadores
ros::Publisher pub_error, pub_switch, pub_backup, pub_info_stop, pub_emergency_stop; 

// ----- Subscriptores
ros::Subscriber sub_navigation, sub_com_teleop, sub_fcn_aux, sub_emergency_stop;  

// ----- Mensajes
Common_files::msg_error msg_err;
Common_files::msg_switch msg_switch;
Common_files::msg_backup msg_backup;
Common_files::msg_info_stop msg_info_stop;
Common_files::msg_emergency_stop msg_emergency_stop;

ConduccionThread * conduccion;
CANCommunication * can;

bool finDePrograma;                     // Flag que comprueba si se crea bien la comunicacion con CAN
int CANflag;                            // Flag contador de reintentos de establecimiento de comunicaciones CAN
bool parada_emergencia;                 // Flag que controla que cuando la parada de emergencia esta ON no pueda recibir mensajes de com_teleop
short valor_conmutador;                 // Flag que contrala el cambio del conmutador de manual a remote
short valor_parada_obstaculo;           // Flag que contrala el cambio de la parada de emergencia por obstaculo
short valor_parada_local;               // Flag que contrala el cambio de la parada de emergencia local
short valor_parada_remote;              // Flag que contrala el cambio de la parada de emergencia remota
short error_a_p;                        // Flag que controla el error del arranque y la parada
short error_acelerador;                  // Flag que controla el error del acelerador
short error_freno_estacionamiento;      // Flag que controla el error del freno de estacionamiento
short error_freno_servicio;             // Flag que controla el error del freno de servicio
short error_cambio_marcha;              // Flag que controla el error del cambio de amrcha
short error_direccion;                  // Flag que controla el error de la direccion
short error_diferenciales;              // Flag que controla el error de los diferenciales



/*******************************************************************************
 *******************************************************************************
 *                          MÉTODOS Y FUNCIONES
 * *****************************************************************************
 * ****************************************************************************/

//Funciones de publicacion
void publishEmergencyStop();
void publishBackup();
void publishSwitch();
//void publishInfoStopOsbtacule (short valor);
//void publishInfoStopLocal (short valor);
//void publishInfoStopRemote (short valor);
void publishInfoStop (short valor, int i);
void publishError (short valor, int i);

//Funciones de suscripcion
void fcn_sub_navigation(const Common_files::msg_navigation);
void fcn_sub_com_teleop(const Common_files::msg_com_teleop);
void fcn_sub_engine_brake(const Common_files::msg_fcn_aux);
void fcn_sub_emergency_stop(const Common_files::msg_emergency_stop);

//Funciones propias
bool createCommunication();
bool disconnectCommunication();
void inicializa_variables();
void checkEmergencyStop();
void checkSwitch();
void checkInfoStop();
void checkError();

// Funciones tratamiento de señales
void do_exit(int);              // what has to be done at program exit
void signal_handler(int);       // the signal handler for manual break Ctrl-C
void init_signals();            // what has to be done at program start