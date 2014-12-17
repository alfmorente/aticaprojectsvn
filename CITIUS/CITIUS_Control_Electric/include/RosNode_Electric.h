
/** 
 * @file  RosNode_Electric.h
 * @brief Declara el tipo de la clase "RosNode_Electric"
 * - La clase implementa la gestión del nodo de conducción (Electric) del 
 * Subsistema de control de UGV
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup ElectricRosNode
 * @{
 */

#ifndef ROSNODE_ELECTRIC_H
#define	ROSNODE_ELECTRIC_H

#include <time.h>
#include "RosNode.h"
#include "ElectricConnectionManager.h"
#include "CITIUS_Control_Electric/msg_electricInfo.h"
#include "CITIUS_Control_Electric/msg_electricCommand.h"
#include "CITIUS_Control_Electric/msg_command.h"
#include "CITIUS_Control_Electric/msg_switcher.h"
#include "CITIUS_Control_Electric/srv_nodeStatus.h"
#include "CITIUS_Control_Electric/srv_vehicleStatus.h"
#include <deque>
#include <cstdlib>
#include "Timer.h"

using namespace std;

/**
 * \class RosNode_Driving
 * \brief Clase que representa al nodo ROS que gestiona la comunicación con el 
 * módulo de alimentación del vehículo
 */
class RosNode_Electric : public RosNode {
private:
  ros::Publisher pubElectricInfo;
  ros::Publisher pubCommand;
  ros::Publisher pubSwitcher;
  ros::Subscriber subsElectricCommand;
  ros::ServiceServer servNodeStatus;
  ros::ServiceClient clientVehicleStatus;
  ElectricConnectionManager *dElectric;
  bool fcv_serv_nodeStatus(CITIUS_Control_Electric::srv_nodeStatus::Request &rq, CITIUS_Control_Electric::srv_nodeStatus::Response &rsp);
public:
  RosNode_Electric();
  ~RosNode_Electric();
  void initROS();
  void fcn_sub_electricCommand(CITIUS_Control_Electric::msg_electricCommand msg);
  ElectricConnectionManager *getDriverMng();
  void publishElectricInfo(ElectricInfo info);
  void checkTurnOff();
  void checkSupplyAlarms();
};

#endif	/* ROSNODE_ELECTRIC_H */

/**
 * @}
 */
