/** 
 * @file  convoy.h
 * @brief Archivo de cabecera del módulo Convoy
 * @author Alfonso Morente
 * @date 19/03/2014
 * @addtogroup Convoy
 * @{
 */

#ifndef CONVOY_H
#define	CONVOY_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

// Mensajes 
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_error.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_mode.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_gps.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_backup.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_module_enable.h"

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
#include <unistd.h>
#include <fcntl.h>

using namespace std;
// Variables globales
#define IP_LOCAL "127.0.0.1"    ///<Dirección IP local
#define IP_REMOTE "127.0.0.1"   ///<Dirección IP del equipo remoto
#define TIMER_LONG 10.0         ///<Tiempo de espera para los "handshake"
#define TIMER_SHORT 1.0         ///<Tiempo de espera para la función "hearbeat"
#define BUFSIZE 2048            ///<Tamaño del buffer de recepción
// Tipos de mensaje transmitidos por el socket
#define TYPE_GPS 0              ///<Mensaje tipo GPS
#define TYPE_ERROR 1            ///<Mensaje tipo ERROR
#define TYPE_MODE 2             ///<Mensaje tipo MODE
#define TYPE_BACKUP 3           ///<Mensaje tipo BACKUP
#define TYPE_HEARTBEAT 4        ///<Mensaje tipo HEARTBEAT
// Definición estados de la variable enableModule
#define ENABLE_INIT 0           ///<Estado inicial del módulo
#define ENABLE_ON 1             ///<Módulo habilitado
#define ENABLE_OFF 2            ///<Desactivación de módulo
// Definición del papel del vehículo (vehicleRole)
#define LEADER 0                ///<Papel vehículo líder
#define FOLLOWER 1              ///<Papel vehículo follower
#define UNDEFINED 2             ///<No se ha recibido todavía que papel desempeña el vehículo

ros::Publisher pub_error;
ros::Publisher pub_mode;
ros::Publisher pub_error_follower;
ros::Publisher pub_backup_follower;
ros::Publisher pub_gps_leader;
ros::Publisher pub_gps_follower;

ros::Subscriber sub_gps;
ros::Subscriber sub_moduleEnable;
ros::Subscriber sub_error;
ros::Subscriber sub_backup;
ros::Subscriber sub_mode_follower;


// Variables globales del módulo
// Variable de continuacion de modulo
bool exitModule;        
short enableModule;
short vehicleRole;
bool initialConnection;         // Controla la conexión inicial del client al server (false=no se realizó la conex inicial)
bool sendDataFlag;              // Bandera para controlar el envío de datos del gps del leader al follower
bool modeACK;                   // Se activa cuando se recibe asentimiento de cambio de modo en GestSistema en Follower
int threadActive;
// Variables para el hilo de escucha por el socket
pthread_t s_Thread;
// Variables globales usadas para la conexion UDP
int fd_local;
struct sockaddr_in local_addr;
struct sockaddr_in remote_addr;
Timer t;
// Mensajes ROS
short type_message;
Common_files::msg_gpsPtr msg_gps(new Common_files::msg_gps);
Common_files::msg_errorPtr msg_error(new Common_files::msg_error);
Common_files::msg_modePtr msg_mode(new Common_files::msg_mode);
Common_files::msg_backupPtr msg_backup(new Common_files::msg_backup);

// Funciones de suscripcion
void fcn_sub_error(const Common_files::msg_errorPtr&);
// Suscriptores del leader
void fcn_sub_gps(const Common_files::msg_gpsPtr&);
void fcn_sub_module_enable(const Common_files::msg_module_enablePtr&);
// Suscriptores del follower
void fcn_sub_mode_follower(const Common_files::msg_modePtr&);
void fcn_sub_backup_follower(const Common_files::msg_backupPtr&);

// Funciones propias
void initialize(ros::NodeHandle n);
bool heartbeat(int, struct sockaddr_in);
bool handshakeLeader();
void disconnectSocket();
bool createSocket(int, struct sockaddr_in);
void convertToROS (unsigned char*);
void* socketThread(void* );
void* clientListener(void* );
bool handshakeFollower();


#endif	/* CONVOY_H */

/**
 *@}
 */