
/** 
 * @file  RosNode_Communication.h
 * @brief Declara el tipo de la clase "RosNode_Communication"
 * - La clase implementa la gestión del nodo de Comunicaciones del 
 * Subsistema de control del UGV
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup CommunicationsRosNode
 * @{
 */

#ifndef COMMUNICATION_H
#define	COMMUNICATION_H

#include "ros/ros.h"
#include <cstdlib>

// Mensajes y servicios subsitema de control
#include "CITIUS_Control_Communication/msg_command.h"
#include "CITIUS_Control_Communication/msg_electricCommand.h"
#include "CITIUS_Control_Communication/msg_ctrlFrontCamera.h"
#include "CITIUS_Control_Communication/msg_ctrlRearCamera.h"
#include "CITIUS_Control_Communication/msg_electricInfo.h"
#include "CITIUS_Control_Communication/msg_vehicleInfo.h"
#include "CITIUS_Control_Communication/msg_posOriInfo.h"
#include "CITIUS_Control_Communication/msg_frontCameraInfo.h"
#include "CITIUS_Control_Communication/msg_rearCameraInfo.h"
#include "CITIUS_Control_Communication/srv_vehicleStatus.h"
#include "CITIUS_Control_Communication/srv_nodeStatus.h"

// Mensajes y servicios subsistema de payload de observacion
#include "CITIUS_Control_Communication/msg_tvinfo.h"
#include "CITIUS_Control_Communication/msg_echoesFound.h"
#include "CITIUS_Control_Communication/msg_irinfo.h"
#include "CITIUS_Control_Communication/msg_panTiltPosition.h"
#include "CITIUS_Control_Communication/msg_lastExec.h"
#include "CITIUS_Control_Communication/srv_dzoom.h"
#include "CITIUS_Control_Communication/srv_polarity.h"
#include "CITIUS_Control_Communication/srv_zoomDirect.h"
#include "CITIUS_Control_Communication/srv_zoomCommand.h"
#include "CITIUS_Control_Communication/srv_focusDirect.h"
#include "CITIUS_Control_Communication/srv_autofocusMode.h"
#include "CITIUS_Control_Communication/srv_panAbsolutePosition.h"
#include "CITIUS_Control_Communication/srv_panRate.h"
#include "CITIUS_Control_Communication/srv_tiltAbsolutePosition.h"
#include "CITIUS_Control_Communication/srv_tiltRate.h"
#include "CITIUS_Control_Communication/srv_shoot.h"

#include "constant.h"
#include "TranslatorROSJAUS.h"
#include "RosNode.h"

// Librerias de JAUS
#include "jaus.h"
#include "openJaus.h"
#include "JausHandler.h"
#include "Timer.h"

using namespace std;

/**
 * \class RosNode_Communications
 * \brief Clase que representa al nodo ROS de gestión de comunicaciones
 */
class RosNode_Communications : public RosNode {
public:
  static RosNode_Communications *getInstance();
  RosNode_Communications();
  ~RosNode_Communications();
  void initROS();
  void initJAUS();
  void finishJAUS();
  void informStatus();
  void informCameraToStream();
  void informHeartbeatPositionInfo();
  bool isControllerAvailable();
private:
  static RosNode_Communications *instance;
  static bool instanceCreated;
  FileLoader *configData;
  JausHandler *handler;
  NodeManager *nm;
  JausSubsystemID subsystemController;
  JausNodeID nodeController;
  short currentObservationCamera;
  short currentDrivingCamera;
  HeartBeatPosition hbPosition;
  ros::Subscriber subsFrontCameraInfo;
  ros::Subscriber subsRearCameraInfo;
  ros::Subscriber subsVehicleInfo;
  ros::Subscriber subsElectricInfo;
  ros::Subscriber subsPosOriInfo;
  ros::Publisher pubCtrlFrontCamera;
  ros::Publisher pubCtrlRearCamera;
  ros::Publisher pubCommand;
  ros::Publisher pubElectricCommand;
  ros::ServiceClient clientStatus;
  ros::Subscriber subsIRCameraInfo;
  ros::Subscriber subsTelemeterInfo;
  ros::Subscriber subsTVCameraInfo;
  ros::Subscriber subsPositionerInfo;
  ros::Subscriber subsLastExec;
  ros::ServiceClient clientIRCameraZoom;
  ros::ServiceClient clientIRCameraPolarity;
  ros::ServiceClient clientTVCameraDirectZoom;
  ros::ServiceClient clientTVCameraContZoom;
  ros::ServiceClient clientTVCameraFocus;
  ros::ServiceClient clientTVCameraAutofocus;
  ros::ServiceClient clientPosPanAbs;
  ros::ServiceClient clientPosPanRate;
  ros::ServiceClient clientPosTiltAbs;
  ros::ServiceClient clientPosTiltRate;
  ros::ServiceClient clientShootTel;
  ros::ServiceServer servNodeStatus;
  OjCmpt missionSpoolerComponent;
  OjCmpt primitiveDriverComponent;
  OjCmpt visualSensorComponent;
  OjCmpt platformSensorComponent;
  OjCmpt velocityStateSensorComponent;
  OjCmpt globalPoseSensorComponent;
  OjCmpt heartBeatInformationComponent;
  static void fcn_receive_run_mission(OjCmpt, JausMessage);
  static void fcn_receive_set_wrench_effort(OjCmpt, JausMessage);
  static void fcn_receive_set_discrete_devices(OjCmpt, JausMessage);
  static void fcn_receive_set_camera_pose(OjCmpt, JausMessage);
  static void fcn_receive_set_signaling_elements(OjCmpt, JausMessage);
  static void fcn_receive_set_positioner(OjCmpt, JausMessage);
  static void fcn_receive_set_day_time_camera(OjCmpt, JausMessage);
  static void fcn_receive_set_night_time_camera(OjCmpt, JausMessage);
  static void fcn_receive_set_telemeter(OjCmpt, JausMessage);
  static void fcn_receive_set_travel_speed(OjCmpt, JausMessage);
  void fnc_subs_frontCameraInfo(CITIUS_Control_Communication::msg_frontCameraInfo msg);
  void fnc_subs_rearCameraInfo(CITIUS_Control_Communication::msg_rearCameraInfo msg);
  void fnc_subs_vehicleInfo(CITIUS_Control_Communication::msg_vehicleInfo msg);
  void fnc_subs_electricInfo(CITIUS_Control_Communication::msg_electricInfo msg);
  void fnc_subs_posOriInfo(CITIUS_Control_Communication::msg_posOriInfo msg);
  void fcn_subs_irCameraInfo(CITIUS_Control_Communication::msg_irinfo msg);
  void fcn_subs_telemeterInfo(CITIUS_Control_Communication::msg_echoesFound msg);
  void fcn_subs_tvCameraInfo(CITIUS_Control_Communication::msg_tvinfo msg);
  void fcn_subs_positionerInfo(CITIUS_Control_Communication::msg_panTiltPosition msg);
  void fcn_subs_lastExec(CITIUS_Control_Communication::msg_lastExec msg);
  bool fcv_serv_nodeStatus(CITIUS_Control_Communication::srv_nodeStatus::Request &rq, CITIUS_Control_Communication::srv_nodeStatus::Response &rsp);
};

#endif	/* COMMUNICATION_H */

/**
 * @}
 */