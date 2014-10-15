
/** 
 * @file  RosNode_FrontCamera.h
 * @brief Declara el tipo de la clase "RosNode_FrontCamera"
 * - La clase implementa la gestión del nodo que controla la camara de apoyo 
 * a la conduccion
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef ROSNODE_FRONTCAMERA_H
#define	ROSNODE_FRONTCAMERA_H

#include <time.h>
#include "RosNode.h"
#include "AxisP3364LveDriver.h"
#include "constant.h"
#include "CITIUS_Control_FrontCamera/msg_ctrlFrontCamera.h"
#include "CITIUS_Control_FrontCamera/msg_frontCameraInfo.h"
#include "CITIUS_Control_FrontCamera/srv_nodeStatus.h"
#include "ros/ros.h"
#include <cstdlib>

using namespace std;

/**
 * /class RosNode_FrontCamera
 * /brief Clase que representa al nodo ROS que gestiona la comunicación con la
 * cámara de apoyo a la conducción
*/
class RosNode_FrontCamera: public RosNode {
private:
  // Publicador de informacion de camara
  ros::Publisher pubFrontCameraInfo;
  // Suscriptor para control de camara
  ros::Subscriber subsCtrlFrontCamera;
  // Servidor de estado de nodo
  ros::ServiceServer servNodeStatus;
  // Driver de la cámara
  AxisP3364LveDriver *dFrontCamera;
  // Callbacks ROS
  void fcn_sub_ctrlFrontCamera(CITIUS_Control_FrontCamera::msg_ctrlFrontCamera msg);
  bool fcv_serv_nodeStatus(CITIUS_Control_FrontCamera::srv_nodeStatus::Request &rq, CITIUS_Control_FrontCamera::srv_nodeStatus::Response &rsp);

public:

  // Constructor
  RosNode_FrontCamera();
  // Inicializador de artefactos ROS
  void initROS();
  // Consultores
  AxisP3364LveDriver *getDriverMng();
  // Metodos de gestion
  void manageDevice();

};

#endif	/* ROSNODE_FRONTCAMERA_H */

/**
 * @}
 */

