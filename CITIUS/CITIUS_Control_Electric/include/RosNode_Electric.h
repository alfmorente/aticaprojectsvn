/* 
 * File:   RosNode_Electric.h
 * Author: Carlos Amores
 *
 * Created on 15 de junio de 2014, 18:28
 */

#ifndef ROSNODE_ELECTRIC_H
#define	ROSNODE_ELECTRIC_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* ROSNODE_ELECTRIC_H */

#include <time.h>
#include "ElectricConnectionManager.h"
#include "constant.h"
#include "CITIUS_Control_Electric/msg_electricInfo.h"
#include "CITIUS_Control_Electric/msg_command.h"
#include "CITIUS_Control_Electric/srv_vehicleStatus.h"

class RosNode_Electric{
private:
    // Estado del nodo
    short emNodeStatus;
    // Publicador de informacion de camara
    ros::Publisher pubElectricInfo;
    ros::Publisher pubCommand;
    // Servidor de estado de nodo
    ros::ServiceClient clientVehicleStatus;
    // Driver de la cámara
    ElectricConnectionManager *dElectric;
public:
    // Constructor
    RosNode_Electric();
    // Inicializador de artefactos ROS
    void initROS();
    // Getter and Setter necesarios
    ros::Publisher getPubElectricInfo();
    ros::ServiceClient getClientVehicleStatus();
    short getEMNodeStatus();
    void setEMNodeStatus(short newEMNodeStatus);
    ElectricConnectionManager *getDriverMng();
    // Gestion de mensajes recibidos
    void manageMessage(FrameDriving frame);
    
};

