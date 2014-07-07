/* 
 * File:   conduccion.h
 * Author: Sergio Doctor López
 *
 * Created on 10 de marzo de 2014, 10:50
 */

#ifndef CONDUCCION_H
#define	CONDUCCION_H

#ifdef __cplusplus
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

// Cabeceras de mensajes de publicacion camion
#include "../../msg_gen/cpp/include/Modulo_Conduccion/messageCAN.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/nivelBomba.h"

// Cabeceras de mensajes de subscripcion
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_navigation.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_com_teleop.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_fcn_aux.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_emergency_stop.h"

// Cabeceras de mensajes de subscripcion camion
#include "../../msg_gen/cpp/include/Modulo_Conduccion/bomba.h"
#include "../../msg_gen/cpp/include/Modulo_Conduccion/mastil.h"


// Cabeceras de servicio ROS
#include "../../../Common_files/srv_gen/cpp/include/Common_files/srv_data.h"


//ROS
#include "ros/ros.h"

//OTROS
#include "../../../Common_files/include/Common_files/constant.h"
#include <stdlib.h>
#include "CANCommunication.hpp"
#include "Thread.hpp"
#include "ConduccionThread.hpp"
#include "ConduccionCamionThread.hpp"
#include "SerialCommunication.hpp"
#include "mastil.h"

// Interaccion con usuario
#include "interaction.h"

#define TIMER 10.0

#define ATICA 1
#define CAMION 2

/*******************************************************************************
 *******************************************************************************
 *                              ATRIBUTOS
 * *****************************************************************************
 * ****************************************************************************/

// Para elegri entre vehiculos
int tipo_vehiculo;

// Para interaccionar con los usuarios
int operationMode;

// ----- Publicadores
ros::Publisher pub_error, pub_switch, pub_backup, pub_info_stop, pub_emergency_stop; 

// ----- Publicador camion de bomberos
ros::Publisher pub_camion, pub_nivel_bomba;

// ----- Subscriptores
ros::Subscriber sub_navigation, sub_com_teleop, sub_fcn_aux, sub_emergency_stop;  

// ----- Subscriptores camion de bomberos
ros::Subscriber subBomba, subMastil;

// ----- Servicios
ros::ServiceServer server;

// ----- Mensajes
//Common_files::msg_error msg_err;
//Common_files::msg_switch msg_switch;
//Common_files::msg_backup msg_backup;
//Common_files::msg_info_stop msg_info_stop;
//Common_files::msg_emergency_stop msg_emergency_stop;

// ----- Mensaje camion de bomberos
Modulo_Conduccion::messageCANPtr msg(new Modulo_Conduccion::messageCAN);
Modulo_Conduccion::nivelBombaPtr msg_mensajeNivel(new Modulo_Conduccion::nivelBomba);

// ----- Mensajes Ptr
Common_files::msg_errorPtr msg_err(new Common_files::msg_error);
Common_files::msg_switchPtr msg_switch(new Common_files::msg_switch);
Common_files::msg_backupPtr msg_backup(new Common_files::msg_backup);
Common_files::msg_info_stopPtr msg_info_stop(new Common_files::msg_info_stop);
Common_files::msg_emergency_stopPtr msg_emergency_stop(new Common_files::msg_emergency_stop);

CANCommunication * can;
ConduccionThread * conduccion;

CANCommunication * canCamion;
ConduccionCamionThread * conduccionCamion;
SerialCommunication *SerialComBomba;
SerialCommunication *SerialComMastil;

bool finDePrograma;                     // Flag que comprueba si se crea bien la comunicacion con CAN
int CANflag;                            // Flag contador de reintentos de establecimiento de comunicaciones CAN
int Serialflag;
bool parada_emergencia;                 // Flag que controla que cuando la parada de emergencia esta ON no pueda recibir mensajes de com_teleop
short valor_conmutador;                 // Flag que contrala el cambio del conmutador de manual a remote
short valor_parada_obstaculo;           // Flag que contrala el cambio de la parada de emergencia por obstaculo
short valor_parada_local;               // Flag que contrala el cambio de la parada de emergencia local
short valor_parada_remote;              // Flag que contrala el cambio de la parada de emergencia remota
short error_a_p;                        // Flag que controla el error del arranque y la parada
short error_acelerador;                 // Flag que controla el error del acelerador
short error_freno_estacionamiento;      // Flag que controla el error del freno de estacionamiento
short error_freno_servicio;             // Flag que controla el error del freno de servicio
short error_cambio_marcha;              // Flag que controla el error del cambio de amrcha
short error_direccion;                  // Flag que controla el error de la direccion
short error_diferenciales;              // Flag que controla el error de los diferenciales

// variables para el camion de bomberos
char * freno_Mano;
double rev;


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
//void fcn_sub_navigation(const Common_files::msg_navigation);
//void fcn_sub_com_teleop(const Common_files::msg_com_teleop);
//void fcn_sub_engine_brake(const Common_files::msg_fcn_aux);
//void fcn_sub_emergency_stop(const Common_files::msg_emergency_stop);
bool fcn_heartbeat(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp);

//Funciones de suscripcion Ptr
void fcn_sub_navigation(const Common_files::msg_navigationPtr&);
void fcn_sub_com_teleop(const Common_files::msg_com_teleopPtr&);
void fcn_sub_engine_brake(const Common_files::msg_fcn_auxPtr&);
void fcn_sub_emergency_stop(const Common_files::msg_emergency_stopPtr&);

//Funciones de suscripcion Ptr Camion de bomberos
void fcn_sub_bomba(const Modulo_Conduccion::bombaPtr&);
void fcn_sub_mastil(const Modulo_Conduccion::mastilPtr&);

//Funciones propias
bool createCommunication();
bool disconnectCommunication();
void initialize(ros::NodeHandle n);
void checkEmergencyStop();
void checkSwitch();
void checkInfoStop();
void checkError();

// Funciones tratamiento de señales
void signal_handler(int);       // the signal handler for manual break Ctrl-C

void convertMessageToROS(struct ksmData ksm, Modulo_Conduccion::messageCANPtr& msg);