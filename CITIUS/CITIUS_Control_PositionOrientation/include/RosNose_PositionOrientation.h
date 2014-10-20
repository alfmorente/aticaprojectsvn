
/** 
 * @file  RosNode_PositionOrientation.h
 * @brief Declara el tipo de la clase "RosNode_PositionOrientation"
 * - La clase implementa la gestión del nodo que controla los dispositivos que
 * proporcionan la posición y orientación del vehículo
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef ROSNOSE_POSITIONORIENTATION_H
#define	ROSNOSE_POSITIONORIENTATION_H

#include "RosNode.h"
#include "CITIUS_Control_PositionOrientation/srv_nodeStatus.h"
#include "CITIUS_Control_PositionOrientation/msg_posOriInfo.h"
#include "constant.h"
#include "XSensMTi700Driver.h"
#include "TraxAHRSModuleDriver.h"
#include <cstdlib>
#include "ros/ros.h"
#include <ros/node_handle.h>

using namespace std;

/**
 * \class RosNode_PositionOrientation
 * \brief Clase que representa al nodo ROS que gestiona la comunicación con los
 * dispositivos que obtiene la posición y orientación del vehículo
*/
class RosNode_PositionOrientation: public RosNode {
  
private:
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
  // Callbacks ROS
  bool fcn_serv_nodeStatus(CITIUS_Control_PositionOrientation::srv_nodeStatus::Request &, CITIUS_Control_PositionOrientation::srv_nodeStatus::Response &);
public:
  // Constructor
  RosNode_PositionOrientation();
  // Destructor
  ~RosNode_PositionOrientation();
  // Inicializador de artefactos ROS
  void initROS();
  // Getter and Setter necesarios
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

#endif	/* ROSNOSE_POSITIONORIENTATION_H */

/**
 * @}
 */