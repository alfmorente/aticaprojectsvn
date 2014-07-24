/* 
 * File:   RosNode_Driving.h
 * Author: Carlos Amores
 *
 * Created on 15 de junio de 2014, 18:28
 */

#ifndef ROSNODE_DRIVING_H
#define	ROSNODE_DRIVING_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* ROSNODE_DRIVING_H */

#include <time.h>
#include "DrivingConnectionManager.h"
#include "constant.h"
#include "CITIUS_Control_Driving/msg_command.h"
#include "CITIUS_Control_Driving/msg_vehicleInfo.h"
#include "CITIUS_Control_Driving/srv_nodeStatus.h"

class RosNode_Driving{
private:
    // Estado del nodo
    short vmNodeStatus;
    // Publicador de informacion de camara
    ros::Publisher pubVehicleInfo;
    // Suscriptor para control de camara
    ros::Subscriber subsCommand;
    // Servidor de estado de nodo
    ros::ServiceServer servNodeStatus;
    // Driver de la cámara
    DrivingConnectionManager *dVehicle;
public:
    // Constructor
    RosNode_Driving();
    // Inicializador de artefactos ROS
    void initROS();
    // Callbacks ROS
    void fcn_sub_command(CITIUS_Control_Driving::msg_command msg);
    bool fcv_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp);
    // Getter and Setter necesarios
    ros::Publisher getPubVehicleInfo();
    short getVMNodeStatus();
    void setVMNodeStatus(short newVMNodeStatus);
    DrivingConnectionManager *getDriverMng();
    // Criba de comandos fuera de rango
    bool checkCommand(CITIUS_Control_Driving::msg_command msg);
    // Gestion de mensajes recibidos
    void manageAlarmsMessage(FrameDriving frame);
    // Publicacion de informacion de vehiculo
    void publishDrivingInfo(DrivingInfo);
    
};

