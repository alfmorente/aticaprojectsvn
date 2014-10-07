
/** 
 * @file  RosNode_PositionOrientation.h
 * @brief Declara el tipo de la clase "RosNode_PositionOrientation"
 * - La clase implementa la gestión del nodo que controla los dispositivos que
 * proporcionan la posicion y orientacion del vehiculo
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef ROSNOSE_POSITIONORIENTATION_H
#define	ROSNOSE_POSITIONORIENTATION_H

#include "CITIUS_Control_PositionOrientation/srv_nodeStatus.h"
#include "CITIUS_Control_PositionOrientation/msg_posOriInfo.h"
#include "constant.h"
#include "XSensMTi700Driver.h"
#include "TraxAHRSModuleDriver.h"
#include <cstdlib>
#include "ros/ros.h"
#include <ros/node_handle.h>

using namespace std;

#endif	/* ROSNOSE_POSITIONORIENTATION_H */

class RosNode_PositionOrientation {
private:
  // Estado del nodo
  short poNodeStatus;
  // Publicador de informacion de Posicion/Orientacion
  ros::Publisher pubPosOriInfo;
  // Cliente de servicio de estado de nodo
  ros::ServiceServer servNodeStatus;
  // Driver de GPS/INS (XSens MTi-G 700)
  XSensMTi700Driver *gpsinsDriver;
  // Driver de Magnetometro (PNI TRAX AHRS Module)
  TraxAHRSModuleDriver *magnetometerDriver;
  // Flags de actividad
  bool magnOK;
  bool gpsinsOK;

public:
  // Constructor
  RosNode_PositionOrientation();
  // Destructor
  ~RosNode_PositionOrientation();

  // Inicializador de artefactos ROS
  void initROS();

  // Callbacks ROS
  bool fcn_serv_nodeStatus(CITIUS_Control_PositionOrientation::srv_nodeStatus::Request &, CITIUS_Control_PositionOrientation::srv_nodeStatus::Response &);

  // Getter and Setter necesarios
  //ros::Publisher getPubPosOriInfo();
  short getPONodeStatus();
  void setPONodeStatus(short);
  XSensMTi700Driver *getXSensManager();
  TraxAHRSModuleDriver *getMagnetometerManager();
  bool getGpsStatus();
  bool getMagnStatus();
  void setGpsStatus(bool);
  void setMagnStatus(bool);

  // Publicador de la informacion
  void publishInformation();

  // Configuracion de dispositivos
  void configureDevices();
};



