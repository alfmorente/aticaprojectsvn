
/** 
 * @file  RosNode_Driving.h
 * @brief Declara el tipo de la clase "RosNode_Driving"
 * - La clase implementa la gestión del nodo de conduccion (Driving) del 
 * Subsistema de control de UGV
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef ROSNODE_DRIVING_H
#define	ROSNODE_DRIVING_H

#include <time.h>
#include "DrivingConnectionManager.h"
//#include "constant.h"
#include "CITIUS_Control_Driving/msg_command.h"
#include "CITIUS_Control_Driving/msg_vehicleInfo.h"
#include "CITIUS_Control_Driving/srv_nodeStatus.h"

#endif	/* ROSNODE_DRIVING_H */

class RosNode_Driving {
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
  short getVMNodeStatus();
  void setVMNodeStatus(short newVMNodeStatus);
  DrivingConnectionManager *getDriverMng();
  
  // Criba de comandos fuera de rango
  bool checkCommand(CITIUS_Control_Driving::msg_command msg);
 
  // Publicacion de informacion de vehiculo
  void publishDrivingInfo(DrivingInfo);

};

