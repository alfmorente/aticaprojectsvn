
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
#include "Driving_Bobcat/msg_command.h"
#include "Driving_Bobcat/msg_switcher.h"
#include "Driving_Bobcat/msg_vehicleInfo.h"
#include "Driving_Bobcat/srv_vehicleStatus.h"
#include "Driving_Bobcat/srv_nodeStatus.h"
#include "Timer.h"

#define MAX_INTERVAL_100 100 ///<Valor máximo de intervalo 0..100
#define MIN_INTERVAL_100 0 ///<Valor mínimo de intervalo 0..100
#define MAX_INTERVAL_200 100 ///<Valor máximo de intervalo -100..100
#define MIN_INTERVAL_200 -100 ///<Valor mínimo de intervalo -100..100
#define MAX_INTERVAL_1000 1000 ///<Valor máximo de intervalo 0..1000
#define MIN_INTERVAL_1000 0 ///<Valor mínimo de intervalo 0..1000
#define PERCENT 5 ///<Umbral de criba para cambios en el valor de los actuadores
#define PERCENT_INTERVAL(min,max)((abs(max-min))*((double)PERCENT/100)) ///<Macro para calcular el valor de criba en función del porcentaje y el intervalo
 
/**
 * \class RosNode_Driving
 * \brief Clase que representa al nodo ROS que gestiona la comunicación con el 
 * módulo de conducción del vehículo
 */
class RosNode_Driving : public RosNode {
private:
  ros::Publisher pubVehicleInfo;
  ros::Publisher pubSwitcher;
  ros::Subscriber subsCommand;
  ros::ServiceServer servNodeStatus;
  ros::ServiceClient clientStatus;
  DrivingConnectionManager *dVehicle;
  bool electricAlarms;
  void fcn_sub_command(Driving_Bobcat::msg_command msg);
  bool fcv_serv_nodeStatus(Driving_Bobcat::srv_nodeStatus::Request &rq, Driving_Bobcat::srv_nodeStatus::Response &rsp);
  bool checkCommand(Driving_Bobcat::msg_command msg);
  void setEmergecyCommands();
  bool minimumRequiredInterval(short element, short value);
public:
  RosNode_Driving();
  ~RosNode_Driving();
  void initROS();
  DrivingConnectionManager *getDriverMng();
  void publishDrivingInfo(DrivingInfo);
  void publishSwitcherInfo(short position);
  void checkAlarms();
  void checkSwitcher();
};

#endif	/* ROSNODE_DRIVING_H */

/**
 * @}
 */