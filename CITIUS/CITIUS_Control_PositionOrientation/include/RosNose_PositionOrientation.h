
/** 
 * @file  RosNode_PositionOrientation.h
 * @brief Declara el tipo de la clase "RosNode_PositionOrientation"
 * - La clase implementa la gestión del nodo que controla los dispositivos que
 * proporcionan la posición y orientación del vehículo
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup PositionOrientationRosNode
 * @{
 */

#ifndef ROSNOSE_POSITIONORIENTATION_H
#define	ROSNOSE_POSITIONORIENTATION_H

#include "RosNode.h"
#include "CITIUS_Control_PositionOrientation/srv_nodeStatus.h"
#include "CITIUS_Control_PositionOrientation/msg_posOriInfo.h"
#include "constant.h"
#include "XSensMTi700Driver.h"
#include <cstdlib>
#include "ros/ros.h"
#include <ros/node_handle.h>
#include "Timer.h"

using namespace std;

/**
 * \class RosNode_PositionOrientation
 * \brief Clase que representa al nodo ROS que gestiona la comunicación con los
 * dispositivos que obtiene la posición y orientación del vehículo
 */
class RosNode_PositionOrientation : public RosNode {
private:
  ros::Publisher pubPosOriInfo;
  ros::ServiceServer servNodeStatus;
  XSensMTi700Driver *gpsinsDriver;
  bool gpsinsOK;
  bool fcn_serv_nodeStatus(CITIUS_Control_PositionOrientation::srv_nodeStatus::Request &, CITIUS_Control_PositionOrientation::srv_nodeStatus::Response &);
public:
  RosNode_PositionOrientation();
  ~RosNode_PositionOrientation();
  void initROS();
  XSensMTi700Driver *getXSensManager();
  bool getGpsStatus();
  void setGpsStatus(bool);
  void publishInformation();
  void configureDevices();
};

#endif	/* ROSNOSE_POSITIONORIENTATION_H */

/**
 * @}
 */