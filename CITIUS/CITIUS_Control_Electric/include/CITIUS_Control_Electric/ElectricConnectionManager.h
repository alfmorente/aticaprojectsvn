/* 
 * File:   ElectricConnectionManager.h
 * Author: Carlos Amores
 *
 * Created on 12 de junio de 2014, 18:35
 */

#ifndef ELECTRICCONNECTIONMANAGER_H
#define	ELECTRICCONNECTIONMANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* ELECTRICCONNECTIONMANAGER_H */

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
#include "CITIUS_Control_Electric/constant.h"
#include "CITIUS_Control_Electric/msg_electricInfo.h"
#include "CITIUS_Control_Electric/srv_nodeStatus.h"
#include "CITIUS_Control_Electric/srv_vehicleStatus.h"


class ElectricConnectionManager {
    public:
        // Constructor
        ElectricConnectionManager();
        // Actuacion sobre atributos
        void setNodeStatus(short newStatus);
        short getNodeStatus();
        void setSocketDescriptor(int newSocketDescriptor);
        int getSocketDescriptor();
        ros::Publisher getPublisherElectricInformation();
        // Tratamiento de mensajes procedentes del payload de conduccion
        void messageManager(FrameDriving fd);
        // Callbacks
        bool fcn_serv_nodeStatus(CITIUS_Control_Electric::srv_nodeStatus::Request &rq, CITIUS_Control_Electric::srv_nodeStatus::Response &rsp);
        // Solicitud de información al vehículo
        void reqVehicleInformation();
   private:
        // Estado del nodo ROS
        short nodeStatus;
        // Manejadores de nodos ROS
        ros::NodeHandle nh;
        // Publicadores
        ros::Publisher publisher_electricInformation;
        // Servidores
        ros::ServiceServer server_nodeState;
        // Clientes
        ros::ServiceClient client_vehicleStatus;
        // Socket
        int socketDescriptor;
};

