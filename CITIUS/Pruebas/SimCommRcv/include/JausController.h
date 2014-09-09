/* 
 * File:   JausController.h
 * Author: atica
 *
 * Created on 9 de septiembre de 2014, 12:58
 */

#ifndef JAUSCONTROLLER_H
#define	JAUSCONTROLLER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* JAUSCONTROLLER_H */

// Librerias de JAUS
#include "jaus.h"
#include "openJaus.h"
#include "JausHandler.h"
#include <iostream>

class JausController {
public:
    
    // Patron Singleton
    static JausController *getInstance();
    
    JausController();
    
    // Inicializacion de artefactos ROS/JAUS
    void initJAUS();

    // Generacion de mensaje de estado para Controller
    void informStatus();

    
private:
    
    // Patron Singleton
    static JausController *instance;
    static bool instanceCreated;
    
    // Manejador de JAUS 
    FileLoader *configData;
    JausHandler *handler;
    NodeManager *nm;
    
    // Controlador activo
    int subsystemController;
    int nodeController;

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
    static void fcn_receive_report_mission_status(OjCmpt, JausMessage);
    // Componente Primitive Driver
    static void fcn_receive_report_wrench_effort(OjCmpt,JausMessage);
    static void fcn_receive_report_discrete_devices(OjCmpt,JausMessage);
    static void fcn_receive_ugv_info(OjCmpt,JausMessage);
    // Componente Visual Sensor
    static void fcn_receive_report_camera_pose(OjCmpt,JausMessage);
    static void fcn_receive_report_signaling_elements(OjCmpt,JausMessage);
    static void fcn_receive_report_positioner(OjCmpt,JausMessage);
    static void fcn_receive_report_day_time_camera(OjCmpt,JausMessage);
    static void fcn_receive_report_night_time_camera(OjCmpt,JausMessage);
    // Componente Platform sensor
    static void fcn_receive_telemeter_info(OjCmpt, JausMessage);
    // Componente Velocity State Sensor
    static void fcn_receive_report_travel_speed(OjCmpt,JausMessage);
    static void fcn_receive_report_velocity_state(OjCmpt,JausMessage);
    // Componente Global Pose Sensor
    static void fcn_receive_report_global_pose(OjCmpt,JausMessage);
    static void fcn_receive_additional_gpsins_info(OjCmpt,JausMessage);
    // Componente HeartBeat Information
    static void fcn_receive_heartbeat_channel_state(OjCmpt,JausMessage);
    static void fcn_receive_heartbeat_position_info(OjCmpt,JausMessage);
};