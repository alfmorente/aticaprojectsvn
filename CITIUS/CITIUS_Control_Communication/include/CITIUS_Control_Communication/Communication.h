/* 
 * File:   Communication.h
 * Author: Carlos Amores
 *
 * Created on 10 de junio de 2014, 11:00
 */

#ifndef COMMUNICATION_H
#define	COMMUNICATION_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* COMMUNICATION_H */

  
#include "ros/ros.h"
#include "CITIUS_Control_Communication/msg_command.h"
#include "CITIUS_Control_Communication/msg_ctrlFrontCamera.h"
#include "CITIUS_Control_Communication/msg_ctrlRearCamera.h"
#include "CITIUS_Control_Communication/msg_electricInfo.h"
#include "CITIUS_Control_Communication/msg_vehicleInfo.h"
#include "CITIUS_Control_Communication/msg_posOriInfo.h"
#include "CITIUS_Control_Communication/msg_frontCameraInfo.h"
#include "CITIUS_Control_Communication/msg_rearCameraInfo.h"
#include "CITIUS_Control_Communication/srv_vehicleStatus.h"

class Communications{
public:
  Communications();
  // Get de publicadores ROS
  ros::Publisher getPublisherFrontCamera();
  ros::Publisher getPublisherRearCamera();
  ros::Publisher getPublisherCommand();
  // Callbacks ROS
  void fnc_subs_frontCameraInfo(CITIUS_Control_Communication::msg_frontCameraInfo msg);
  void fnc_subs_rearCameraInfo(CITIUS_Control_Communication::msg_rearCameraInfo msg);
  void fnc_subs_vehicleInfo(CITIUS_Control_Communication::msg_vehicleInfo msg);
  void fnc_subs_electricInfo(CITIUS_Control_Communication::msg_electricInfo msg);
  void fnc_subs_posOriInfo(CITIUS_Control_Communication::msg_posOriInfo msg);
private:
  ros::NodeHandle nh;
  ros::Subscriber subsFrontCameraInfo;
  ros::Subscriber subsRearCameraInfo;
  ros::Subscriber subsVehicleInfo;
  ros::Subscriber subsElectricInfo;
  ros::Subscriber subsPosOriInfo;
  ros::Publisher pubCtrlFrontCamera;
  ros::Publisher pubCtrlRearCamera;
  ros::Publisher pubCommand;
  ros::ServiceClient clientStatus;
};