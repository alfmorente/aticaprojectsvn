
#include "JausController.h"

// Declaracion para patron Singleton
bool JausController::instanceCreated = false;
JausController *JausController::instance = NULL;

using namespace std;

/*******************************************************************************
 *******************************************************************************
 *                            PATRON SINGLETON                                 *
 *******************************************************************************
 ******************************************************************************/

JausController *JausController::getInstance(){
    if(!instanceCreated){ 
        instance = new JausController();
        instanceCreated = true;
    }
    return instance;
}


JausController::JausController() {

}

/*******************************************************************************
 *******************************************************************************
 *                     INICIALIZACION DE ARTEFACTOS JAUS                        *
 *******************************************************************************
 ******************************************************************************/

void JausController::initJAUS() {
    
    // Inicializacion de JAUS
    configData = new FileLoader("nodeManager.conf");
    handler = new JausHandler();
    
    try {
        
        configData = new FileLoader("nodeManager.conf");
        handler = new JausHandler();
        nm = new NodeManager(this->configData, this->handler);
        
    }catch(...){
        
        cout << "Modulo JAUS iniciado" << endl;
                
    }
    
    /*
     * Creacion de componentes
     * 
     */
    
    // Mission Spooler
    missionSpoolerComponent = ojCmptCreate((char *) "Mission Spooler", JAUS_MISSION_SPOOLER, 1);
    if (missionSpoolerComponent == NULL) {
        
        cout << "No se ha podido crear el componente MISSION SPOOLER" << endl;
        exit(0);
        
    }else{
        
        // Mensajes que recibe
        ojCmptAddServiceInputMessage(missionSpoolerComponent, JAUS_MISSION_SPOOLER, JAUS_REPORT_MISSION_STATUS, 0xFF);
        
        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(missionSpoolerComponent, JAUS_REPORT_MISSION_STATUS, fcn_receive_report_mission_status);
        
    }
    
    // Primitive Driver
    primitiveDriverComponent = ojCmptCreate((char *) "Primitive Driver", JAUS_PRIMITIVE_DRIVER, 1);
    if (primitiveDriverComponent == NULL) {
        
        cout << "No se ha podido crear el componente PRIMITIVE DRIVER" << endl;
        exit(0);
    
    }else{

        // Mensajes que recibe
        ojCmptAddServiceInputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_WRENCH_EFFORT, 0xFF);
        ojCmptAddServiceInputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_DISCRETE_DEVICES, 0xFF);
        ojCmptAddServiceInputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_UGV_INFO_12, 0xFF);
        
        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(primitiveDriverComponent, JAUS_REPORT_WRENCH_EFFORT, fcn_receive_report_wrench_effort);
        ojCmptSetMessageCallback(primitiveDriverComponent, JAUS_REPORT_DISCRETE_DEVICES, fcn_receive_report_discrete_devices);
        ojCmptSetMessageCallback(primitiveDriverComponent, JAUS_UGV_INFO_12, fcn_receive_ugv_info);
    }
    
    // Visual Sensor
    visualSensorComponent = ojCmptCreate((char *) "Visual Sensor", JAUS_VISUAL_SENSOR, 1);
    if (visualSensorComponent == NULL) {
        
        cout << "No se ha podido crear el componente VISUAL SENSOR" << endl;
        exit(0);
        
    }else{
        
        // Mensajes que recibe

        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_CAMERA_POSE, 0xFF);
        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_SIGNALING_ELEMENTS_25, 0xFF);
        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_POSITIONER_20, 0xFF);
        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_DAY_TIME_CAMERA_22, 0xFF);
        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_NIGHT_TIME_CAMERA_24, 0xFF);
        
        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_REPORT_CAMERA_POSE, fcn_receive_report_camera_pose);
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_REPORT_SIGNALING_ELEMENTS_25, fcn_receive_report_signaling_elements);
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_REPORT_POSITIONER_20, fcn_receive_report_positioner);
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_REPORT_DAY_TIME_CAMERA_22, fcn_receive_report_day_time_camera);
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_REPORT_NIGHT_TIME_CAMERA_24, fcn_receive_report_night_time_camera);
    
    }
    
    // Platform Sensor
    platformSensorComponent = ojCmptCreate((char *) "Platform Sensor", JAUS_PLATFORM_SENSOR, 1);
    if (platformSensorComponent == NULL) {
        
        cout << "No se ha podido crear el componente PLATFORM SENSOR" << endl;
        exit(0);
        
    }else{

        // Mensajes que recibe
        ojCmptAddServiceInputMessage(platformSensorComponent, JAUS_PLATFORM_SENSOR, JAUS_TELEMETER_INFO_10, 0xFF);
        
        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(platformSensorComponent, JAUS_TELEMETER_INFO_10, fcn_receive_telemeter_info);
        
    }
    
    // Global Waypoint Driver
    globalWaypointDriverComponent = ojCmptCreate((char *)"Global Waypoint Driver", JAUS_GLOBAL_WAYPOINT_DRIVER,1);
    if (globalWaypointDriverComponent == NULL) {
        
        cout << "No se ha podido crear el componente GLOBAL WAYPOINT DRIVER" << endl;
        exit(0);
        
    }else{
        
        // REVISAR COMPONENTE INNECESARIO PARA UGV
        // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
    
    // Velocity State Sensor
    velocityStateSensorComponent = ojCmptCreate((char *)"Velocity State Sensor",JAUS_VELOCITY_STATE_SENSOR,1);
    if (velocityStateSensorComponent == NULL) {
        
        cout << "No se ha podido crear el componente VELOCITY STATE SENSOR" << endl;
        exit(0);
        
    }else{

        // Mensajes que recibe
        ojCmptAddServiceInputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_REPORT_TRAVEL_SPEED, 0xFF);
        ojCmptAddServiceInputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_REPORT_VELOCITY_STATE, 0xFF);

        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(velocityStateSensorComponent, JAUS_REPORT_TRAVEL_SPEED, fcn_receive_report_travel_speed);
        ojCmptSetMessageCallback(velocityStateSensorComponent, JAUS_REPORT_VELOCITY_STATE, fcn_receive_report_velocity_state);
    
    }
    
    // Global Pose Sensor
    globalPoseSensorComponent = ojCmptCreate((char *)"Global Pose Sensor",JAUS_GLOBAL_POSE_SENSOR,1);
    if (globalPoseSensorComponent == NULL) {
        
        cout << "No se ha podido crear el componente GLOBAL POSE SENSOR" << endl;
        exit(0);
        
    }else{
        
        // Mensajes que recibe
        ojCmptAddServiceInputMessage(globalPoseSensorComponent, JAUS_GLOBAL_POSE_SENSOR, JAUS_REPORT_GLOBAL_POSE, 0xFF);
        ojCmptAddServiceInputMessage(globalPoseSensorComponent, JAUS_GLOBAL_POSE_SENSOR, JAUS_ADITIONAL_GPSINS_INFO_4, 0xFF);
        
         // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(globalPoseSensorComponent, JAUS_REPORT_GLOBAL_POSE, fcn_receive_report_global_pose);
        ojCmptSetMessageCallback(globalPoseSensorComponent, JAUS_ADITIONAL_GPSINS_INFO_4, fcn_receive_additional_gpsins_info);
    }
    
    // HeartBeat Information
    heartBeatInformationComponent = ojCmptCreate((char *)"HeartBeat Information", JAUS_HEARTBEAT_INFORMATION,1);
    if (heartBeatInformationComponent == NULL) {
        
        cout << "No se ha podido crear el componente HEARTBEAT INFORMATION" << endl;
        exit(0);
        
    }else{
        // Mensajes que recibe
        ojCmptAddServiceInputMessage(heartBeatInformationComponent, JAUS_HEARTBEAT_INFORMATION, JAUS_HEARTBEAT_CHANNEL_STATE_16, 0xFF);
        ojCmptAddServiceInputMessage(heartBeatInformationComponent, JAUS_HEARTBEAT_INFORMATION, JAUS_HEARTBEAT_POSITION_INFO_17, 0xFF);

        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(heartBeatInformationComponent, JAUS_HEARTBEAT_CHANNEL_STATE_16, fcn_receive_heartbeat_channel_state);
        ojCmptSetMessageCallback(heartBeatInformationComponent, JAUS_HEARTBEAT_POSITION_INFO_17, fcn_receive_heartbeat_position_info);
    
    }
}

/*******************************************************************************
 *******************************************************************************
 *                              CALLBACKS JAUS                                 *
 *******************************************************************************
 ******************************************************************************/

// Componente Mission Spooler

void JausController::fcn_receive_report_mission_status(OjCmpt comp, JausMessage msg) {

    cout << "Recibido mensaje REPORT MISSION STATUS" << endl;

    ReportMissionStatusMessage rMission = reportMissionStatusMessageFromJausMessage(msg);
    cout << "    Status: " << rMission->status << endl;

    reportMissionStatusMessageDestroy(rMission);
}

// Componente Primitive Driver

void JausController::fcn_receive_report_wrench_effort(OjCmpt comp, JausMessage msg) {

}

void JausController::fcn_receive_report_discrete_devices(OjCmpt comp, JausMessage msg) {

}

void JausController::fcn_receive_ugv_info(OjCmpt comp, JausMessage msg) {

}

// Componente Visual Sensor

void JausController::fcn_receive_report_camera_pose(OjCmpt comp, JausMessage msg) {

}

void JausController::fcn_receive_report_signaling_elements(OjCmpt comp, JausMessage msg) {

}

void JausController::fcn_receive_report_positioner(OjCmpt comp, JausMessage msg) {

}

void JausController::fcn_receive_report_day_time_camera(OjCmpt comp, JausMessage msg) {

}

void JausController::fcn_receive_report_night_time_camera(OjCmpt comp, JausMessage msg) {

}

// Componente Platform sensor

void JausController::fcn_receive_telemeter_info(OjCmpt comp, JausMessage msg) {

}

// Componente Velocity State Sensor

void JausController::fcn_receive_report_travel_speed(OjCmpt comp, JausMessage msg) {

}

void JausController::fcn_receive_report_velocity_state(OjCmpt comp, JausMessage msg) {

}

// Componente Global Pose Sensor

void JausController::fcn_receive_report_global_pose(OjCmpt comp, JausMessage msg) {

}

void JausController::fcn_receive_additional_gpsins_info(OjCmpt comp, JausMessage msg) {

}

// Componente HeartBeat Information

void JausController::fcn_receive_heartbeat_channel_state(OjCmpt comp, JausMessage msg) {

}

void JausController::fcn_receive_heartbeat_position_info(OjCmpt comp, JausMessage msg) {

}