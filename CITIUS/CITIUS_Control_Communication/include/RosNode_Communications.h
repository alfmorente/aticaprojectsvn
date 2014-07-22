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
#include "constant.h"
#include "TranslatorROSJAUS.h"

// Librerias de JAUS
#include "jaus.h"
#include "openJaus.h"
#include "JausHandler.h"

class RosNode_Communications {
public:
    RosNode_Communications();
    
    // Inicializacion de artefactos ROS/JAUS
    void initROS();
    void initJAUS();
    
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
    // Generacion de mensaje de estado para Controller
    void informStatus();
private:
    // Manejador de JAUS 
    FileLoader *configData;
    JausHandler *handler;
    NodeManager *nm;
    
    // Controlador activo
    int subsystemController;
    int nodeController;
    
    // Artefactos ROS
    ros::Subscriber subsFrontCameraInfo;
    ros::Subscriber subsRearCameraInfo;
    ros::Subscriber subsVehicleInfo;
    ros::Subscriber subsElectricInfo;
    ros::Subscriber subsPosOriInfo;
    ros::Publisher pubCtrlFrontCamera;
    ros::Publisher pubCtrlRearCamera;
    ros::Publisher pubCommand;
    ros::ServiceClient clientStatus;
    
    // Componentes JAUS
    OjCmpt missionSpoolerComponent;
    OjCmpt primitiveDriverComponent;
    OjCmpt visualSensorComponent;
    OjCmpt platformSensorComponent;
    OjCmpt globalWaypointDriverComponent;
    OjCmpt velocityStateSensorComponent;
    OjCmpt globalPoseSensorComponent;
    OjCmpt heartBeatInformationComponent;
    
    // Callbacks JAUS
    // Componente Mission Spooler
    static void fcn_receive_run_mission(OjCmpt, JausMessage);
    // Componente Primitive Driver
    static void fcn_receive_set_wrench_effort(OjCmpt,JausMessage);
    static void fcn_receive_set_discrete_devices(OjCmpt,JausMessage);
    // Componente Visual Sensor
    static void fcn_receive_set_camera_pose(OjCmpt,JausMessage);
    static void fcn_receive_set_signaling_elements(OjCmpt,JausMessage);
    static void fcn_receive_set_positioner(OjCmpt,JausMessage);
    static void fcn_receive_set_day_time_camera(OjCmpt,JausMessage);
    static void fcn_receive_set_night_time_camera(OjCmpt,JausMessage);
    // Componente Velocity State Sensor
    static void fcn_receive_set_travel_speed(OjCmpt,JausMessage);
    // Componente HeartBeat Information
    static void fcn_receive_heartbeat_channel_state(OjCmpt,JausMessage);
    static void fcn_receive_heartbeat_position_info(OjCmpt,JausMessage);
};