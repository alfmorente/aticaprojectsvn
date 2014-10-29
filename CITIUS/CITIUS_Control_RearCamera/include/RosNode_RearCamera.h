
/** 
 * @file  RosNode_RearCamera.h
 * @brief Declara el tipo de la clase "RosNode_RearCamera"
 * - La clase implementa la gestión del nodo que controla la camara de apoyo 
 * a la conduccion
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup RearCameraRosNode
 * @{
 */

#ifndef ROSNODE_REARCAMERA_H
#define	ROSNODE_REARCAMERA_H

#include "RosNode.h"
#include <time.h>
#include "AxisP3364LveDriver.h"
#include "constant.h"
#include "CITIUS_Control_RearCamera/msg_ctrlRearCamera.h"
#include "CITIUS_Control_RearCamera/msg_rearCameraInfo.h"
#include "CITIUS_Control_RearCamera/srv_nodeStatus.h"
#include "ros/ros.h"
#include <cstdlib>

/**
 * \class RosNode_RearCamera
 * \brief Clase que representa al nodo ROS que gestiona la comunicación con la
 * cámara de apoyo a la conducción
*/
class RosNode_RearCamera: public RosNode {
private:
  // Publicador de informacion de camara
  ros::Publisher pubRearCameraInfo;
  // Suscriptor para control de camara
  ros::Subscriber subsCtrlRearCamera;
  // Servidor de estado de nodo
  ros::ServiceServer servNodeStatus;
  // Driver de la cámara
  AxisP3364LveDriver *dRearCamera;
  // Callbacks ROS
  void fcn_sub_ctrlRearCamera(CITIUS_Control_RearCamera::msg_ctrlRearCamera msg);
  bool fcv_serv_nodeStatus(CITIUS_Control_RearCamera::srv_nodeStatus::Request &rq, CITIUS_Control_RearCamera::srv_nodeStatus::Response &rsp);
public:
  // Constructor
  RosNode_RearCamera();
  // Destructor
  ~RosNode_RearCamera();
  // Inicializador de artefactos ROS
  void initROS();
  // Consultores
  AxisP3364LveDriver *getDriverMng();
  // Metodos de gestion
  void manageDevice();
};

#endif	/* ROSNODE_REARCAMERA_H */

/**
 * @}
 */