/* 
 * File:   convoy.h
 * Author: atica
 *
 * Created on 19 de marzo de 2014, 9:51
 */

#ifndef CONVOY_H
#define	CONVOY_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* CONVOY_H */

// Mensajes 
#include "Common_files/msg_error.h"
#include "Common_files/msg_mode.h"
#include "Common_files/msg_gps.h"
#include "Common_files/msg_backup.h"
#include "Common_files/msg_module_enable.h"

//ROS y demás librerias
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pwd.h>
#include "../../../Common_files/include/Common_files/constant.h"
#include "interaction.h"
#include "Timer.hpp"

using namespace std;

#define SERVER_IP "181.0.0.1"
#define CLIENT_IP "181.0.0.2"
#define TIMER 10.0
#define BUFSIZE 2048
// Tipos de mensaje de la struct ROSmessage
#define TYPE_GPS 0
#define TYPE_ERROR 1
#define TYPE_MODE 2
#define TYPE_BACKUP 3
// Definición estados de la variable enableModule
#define ENABLE_INIT 0           // Estado inicial del módulo
#define ENABLE_ON 1             // Módulo habilitado
#define ENABLE_OFF 2            // Desactivación de módulo
// Definición del papel del vehículo (vehicleRole)
#define LEADER 0
#define FOLLOWER 1

// Variables globales del módulo
// Variable de continuacion de modulo
bool exitModule;        
short enableModule;
short vehicleRole;
bool initialConnection;         // Controla la conexión inicial del client al server (false=no se realizó la conex inicial)
bool sendDataFlag;              // Bandera para controlar el envío de datos del gps del leader al follower
// Variables globales usadas para la conexion UDP
int fd_client;
int fd_server;
struct sockaddr_in client_addr;
struct sockaddr_in server_addr;
Timer t;
// Variables de conversion de mensajes ROS
Common_files::msg_mode conv_msg_mode;
// Estructura para conversión socket-ROS
typedef struct{
    short type_message;
    Common_files::msg_gps gps;
    Common_files::msg_error error;
    Common_files::msg_mode mode;
    Common_files::msg_backup backup;
}ROSmessage;

// Funciones de suscripcion
// Suscriptores del leader
void fcn_sub_gps(const Common_files::msg_gps);
void fcn_sub_module_enable(const Common_files::msg_module_enable);
void fcn_sub_error_leader(const Common_files::msg_error);
// Suscriptores del follower
void fcn_sub_mode_follower(const Common_files::msg_mode);
void fcn_sub_error_follower(const Common_files::msg_error);
void fcn_sub_backup_follower(const Common_files::msg_backup);

// Funciones propias
void initialize(ros::NodeHandle n);
bool handshake();
void disconnectClient();
ROSmessage convertToROS (unsigned char*);