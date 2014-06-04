/* 
 * File:   ConnectionManager.h
 * Author: Carlos Amores
 *
 * Created on 27 de mayo de 2014, 12:15
 */

#ifndef CONNECTIONMANAGER_H
#define	CONNECTIONMANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* CONNECTIONMANAGER_H */

/*******************************************************************************
 *                              LIBRERIAS
*******************************************************************************/
#include <termios.h>
#include <fcntl.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <strings.h>
#include <string>
#include <string.h>
#include "ros/ros.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sstream>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "CITIUS_Control_Driving/msg_vehicleInformation.h"
#include "CITIUS_Control_Driving/msg_command.h"
#include "CITIUS_Control_Driving/srv_nodeStatus.h"


/*******************************************************************************
 *              CONSTANTES IDENTIFICADORES DE DISPOSITIVOS
*******************************************************************************/

#define ID_BLINKER_RIGHT 0
#define ID_BLINKER_LEFT 1
#define ID_BLINKER_EMERGENCY 2
#define ID_DIPSP 3
#define ID_DIPSS 4
#define ID_DIPSR 5
#define ID_KLAXON 6
#define ID_GEAR 7
#define ID_THROTTLE 8
#define ID_MOTOR_RPM 9
#define ID_CRUISING_SPEED 10 
#define ID_MOTOR_TEMPERATURE 11
#define ID_HANDBRAKE 12
#define ID_BRAKE 13
#define ID_STEERING 14
#define ID_ALARMS 15

/*******************************************************************************
 *              CONSTANTES CONMUTACION MANUAL/TELEMANDADO
*******************************************************************************/

#define MT_BLINKERS 16
#define MT_LIGHTS 17
#define MT_THROTTLE 18
#define MT_BRAKE 19
#define MT_HANDBRAKE 20
#define MT_STEERING 21
#define MT_GEAR 22

/*******************************************************************************
 *              MAXIMOS/MINIMOS PARA EL AJUSTE DE VALORES
*******************************************************************************/

#define MAX_BOOLEAN 1
#define MIN_BOOLEAN 0
#define MAX_GEAR 2
#define MIN_GEAR 0
#define MAX_PERCENT 100
#define MIN_PERCENT 0
#define MAX_STEERING 100
#define MIN_STEERING -100
#define MAX_RPM_SPD 1000
#define MIN_RPM_SPD 0
#define MAX_TEMPERATURE 400
#define MIN_TEMPERATURE 0

/*******************************************************************************
 *                              ESTADOS DEL NODO
*******************************************************************************/

#define NODESTATUS_INIT 0
#define NODESTATUS_OK 1
#define NODESTATUS_CORRUPT 2
#define NODESTATUS_OFF 3

/*******************************************************************************
 *                   SOCKET PAYLOAD DE CONDUCCION
*******************************************************************************/

#define IP_PAYLOAD_CONDUCCION "localhost"
#define PORT_PAYLOAD_CONDUCCION 5000

/*******************************************************************************
 *           ESTRUCTURA DE INTERCAMBIO CON PAYLOAD DE CONDUCCION
*******************************************************************************/

typedef struct{
    short id_device;
    short value;
}FrameDriving;

/*******************************************************************************
 *              CLASE MANEJADOR DE CONEXION CON DRIVING
*******************************************************************************/

class DrivingConnectionManager{
    public:
        // Constructor
        DrivingConnectionManager();        
        // Actuacion sobre atributos
        void setNodeStatus(short newStatus);
        short getNodeStatus();
        void setSocketDescriptor(int newSocketDescriptor);
        int getSocketDescriptor();
        ros::Publisher getPublisherVehicleInformation();
        // Callbacks
        void fnc_subs_command(CITIUS_Control_Driving::msg_command msg);
        bool fcn_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp);

    private:
        // Estado del nodo ROS
        short nodeStatus;
        // Manejadores de nodos ROS
        ros::NodeHandle nh;
        // Publicadores
        ros::Publisher publisher_vehicleInformation;
        // Suscriptores
        ros::Subscriber subscriber_command;
        // Servidores
        ros::ServiceServer server_nodeState;
        // Socket
        int socketDescriptor;
        // Estructura de recepcion
        FrameDriving recvFrame;
};