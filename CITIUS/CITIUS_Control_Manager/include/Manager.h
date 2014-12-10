
/** 
 * @file  Manager.h
 * @brief Declara el tipo de la clase "Manager"
 * - La clase implementa la máquina de estados del vehículo y gestiona las 
 * peticiones de transición que puedan producirse. Gestiona la puesta en marcha
 * y final del estado de los nodos ROS secundarios
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Manager
 * @{
 */

#ifndef MANAGER_H
#define	MANAGER_H

#include "ros/ros.h"
#include "CITIUS_Control_Manager/srv_nodeStatus.h"
#include "CITIUS_Control_Manager/srv_vehicleStatus.h"
#include "CITIUS_Control_Manager/msg_switcher.h"
#include "CITIUS_Control_Manager/msg_lastExec.h"
#include "constant.h"
#include "TurnOffAlright.h"
#include <cstdlib>

using namespace std;

/**
 * \class Manager
 * \brief Clase que representa al nodo ROS Manager
 */
class Manager {
public:
  Manager();
  ~Manager();
  void initROS();
  void checkPreviousExec();
private:
  TurnOffAlright *turnOffChecker;
  bool communication;
  bool positionOrientationOK;
  bool frontCameraOK;
  bool rearCameraOK;
  bool drivingOK;
  bool irCameraOK;
  bool lrfOK;
  bool tvCameraOK;
  bool positionerOK;
  short currentSwitcher;
  ros::ServiceClient cmNodeStatus;
  ros::ServiceClient poNodeStatus;
  ros::ServiceClient fcNodeStatus;
  ros::ServiceClient rcNodeStatus;
  ros::ServiceClient drNodeStatus;
  ros::ServiceClient irNodeStatus;
  ros::ServiceClient lrfNodeStatus;
  ros::ServiceClient tvNosdeStatus;
  ros::ServiceClient ptNodeStatus;
  ros::ServiceClient switcher;
  ros::ServiceServer serverVehicleStatus;
  ros::Subscriber switcherLocalTelecontrol;
  ros::Publisher pubLastExec;
  bool fcv_serv_vehicleStatus(CITIUS_Control_Manager::srv_vehicleStatus::Request &rq, CITIUS_Control_Manager::srv_vehicleStatus::Response &rsp);
  void fnc_subs_switcher(CITIUS_Control_Manager::msg_switcher);
};

#endif	/* MANAGER_H */

/**
 * @}
 */