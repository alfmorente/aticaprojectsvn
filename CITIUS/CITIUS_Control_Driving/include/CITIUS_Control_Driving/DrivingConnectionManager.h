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
#include "CITIUS_Control_Driving/msg_vehicleInfo.h"
#include "CITIUS_Control_Driving/msg_command.h"
#include "CITIUS_Control_Driving/srv_nodeStatus.h"
#include "constant.h"


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
        ros::Publisher getPublisherVehicleInfo();
        // Callbacks
        void fnc_subs_command(CITIUS_Control_Driving::msg_command msg);
        bool fcn_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp);
        // Tratamiento de mensajes procedentes del payload de conduccion
        void messageManager(FrameDriving fd);
        // Comprobacion de comando con valores en rangos establecidos
        bool checkCommandInterval(short element, short value);
        // Solicitud de información al vehículo
        void reqVehicleInformation();
    private:
        // Estado del nodo ROS
        short nodeStatus;
        // Manejadores de nodos ROS
        ros::NodeHandle nh;
        // Publicadores
        ros::Publisher publisher_vehicleInfo;
        // Suscriptores
        ros::Subscriber subscriber_command;
        // Servidores
        ros::ServiceServer server_nodeState;
        // Socket
        int socketDescriptor;
};