
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
#include "constant.h"
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
private:
  // Estado de los demas nodos
  bool communication;
  bool positionOrientationOK;
  bool frontCameraOK;
  bool rearCameraOK;
  bool drivingOK;
  bool irCameraOK;
  bool lrfOK;
  bool tvCameraOK;
  bool positionerOK;
  // Posicion del conmutador Local/teleoperado
  short currentSwitcher;
  // Artefactos ROS
  // Estado del nodo Communication
  ros::ServiceClient cmNodeStatus;
  // Estado del nodo Position/Orientation
  ros::ServiceClient poNodeStatus;
  // Estado del nodo FrontCamera
  ros::ServiceClient fcNodeStatus;
  // Estado del nodo RearCamera
  ros::ServiceClient rcNodeStatus;
  // Estado del nodo Driving
  ros::ServiceClient drNodeStatus;
  // Estado del nodo IR Camera
  ros::ServiceClient irNodeStatus;
  // Estado del nodo LRF
  ros::ServiceClient lrfNodeStatus;
  // Estado del nodo TV Camera
  ros::ServiceClient tvNosdeStatus;
  // Estado del nodo Positioner
  ros::ServiceClient ptNodeStatus;
  // Estado del conmutador local/teleoperado (from Electric)
  ros::ServiceClient switcher;
  ros::ServiceServer serverVehicleStatus;
  ros::Subscriber switcherLocalTelecontrol;
  // Callbacks
  bool fcv_serv_vehicleStatus(CITIUS_Control_Manager::srv_vehicleStatus::Request &rq, CITIUS_Control_Manager::srv_vehicleStatus::Response &rsp);
  void fnc_subs_switcher(CITIUS_Control_Manager::msg_switcher);
};

#endif	/* MANAGER_H */

/**
 * @}
 */