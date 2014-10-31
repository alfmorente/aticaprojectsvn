
/** 
 * @file  RosNode_Driving.h
 * @brief Declara el tipo de la clase "RosNode_Driving"
 * - La clase implementa la gestión del nodo de conduccion (Driving) del 
 * Subsistema de control de UGV
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup DrivingRosNode
 * @{
 */

#ifndef ROSNODE_DRIVING_H
#define	ROSNODE_DRIVING_H

#include <time.h>
#include "RosNode.h"
#include "DrivingConnectionManager.h"
#include "CITIUS_Control_Driving/msg_command.h"
#include "CITIUS_Control_Driving/msg_vehicleInfo.h"
#include "CITIUS_Control_Driving/srv_nodeStatus.h"
#include "Timer.h"

/**
 * \class RosNode_Driving
 * \brief Clase que representa al nodo ROS que gestiona la comunicación con el 
 * módulo de conducción del vehículo
*/
class RosNode_Driving: public RosNode {
private:
  // Publicador de informacion de camara
  ros::Publisher pubVehicleInfo;
  // Suscriptor para control de camara
  ros::Subscriber subsCommand;
  // Servidor de estado de nodo
  ros::ServiceServer servNodeStatus;
  // Driver de la cámara
  DrivingConnectionManager *dVehicle;
  // Callbacks ROS
  void fcn_sub_command(CITIUS_Control_Driving::msg_command msg);
  bool fcv_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp);
  // Criba de comandos fuera de rango
  bool checkCommand(CITIUS_Control_Driving::msg_command msg);
public:
  // Constructor
  RosNode_Driving();
  // Destructor
  ~RosNode_Driving();
  // Inicializador de artefactos ROS
  void initROS();
  // Consultores / modificadores de la clase
  DrivingConnectionManager *getDriverMng();
  // Publicacion de informacion de vehiculo
  void publishDrivingInfo(DrivingInfo);
};

#endif	/* ROSNODE_DRIVING_H */

/**
 * @}
 */