
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
class RosNode_Driving : public RosNode {
private:
  ros::Publisher pubVehicleInfo;
  ros::Subscriber subsCommand;
  ros::ServiceServer servNodeStatus;
  DrivingConnectionManager *dVehicle;
  bool electricAlarms;
  void fcn_sub_command(CITIUS_Control_Driving::msg_command msg);
  bool fcv_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp);
  bool checkCommand(CITIUS_Control_Driving::msg_command msg);
  void setEmergecyCommands();
public:
  RosNode_Driving();
  ~RosNode_Driving();
  void initROS();
  DrivingConnectionManager *getDriverMng();
  void publishDrivingInfo(DrivingInfo);
  void checkAlarms();
};

#endif	/* ROSNODE_DRIVING_H */

/**
 * @}
 */