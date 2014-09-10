
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
    cout << "    MIssion ID: " << rMission->missionId << endl;
    cout << "    Status: " << rMission->status << endl;

    cout << " ---- " << endl;
    
    reportMissionStatusMessageDestroy(rMission);
}

// Componente Primitive Driver

void JausController::fcn_receive_report_wrench_effort(OjCmpt comp, JausMessage msg) {
    
    cout << "Recibido mensaje REPORT WRENCH EFFORT" << endl;
    
    ReportWrenchEffortMessage rWrenchEffort = reportWrenchEffortMessageFromJausMessage(msg);
    // Direccion
    if(jausShortIsBitSet(rWrenchEffort->presenceVector,JAUS_WRENCH_PV_PROPULSIVE_ROTATIONAL_Z_BIT)){
        cout << "    Direccion: " << rWrenchEffort->propulsiveRotationalEffortZPercent << endl;
    }
    // Acelerador
    if(jausShortIsBitSet(rWrenchEffort->presenceVector,JAUS_WRENCH_PV_PROPULSIVE_LINEAR_X_BIT)){
        cout << "    Acelerador: " << rWrenchEffort->propulsiveLinearEffortXPercent << endl;
    }
    // Freno de servicio
    if(jausShortIsBitSet(rWrenchEffort->presenceVector,JAUS_WRENCH_PV_RESISTIVE_LINEAR_X_BIT)){
        cout << "    Freno de servicio: " << rWrenchEffort->resistiveLinearEffortXPercent << endl;
    }
    reportWrenchEffortMessageDestroy(rWrenchEffort);
    cout << " ---- " << endl;
}

void JausController::fcn_receive_report_discrete_devices(OjCmpt comp, JausMessage msg) {
    
    cout << "Recibido mensaje REPORT DISCRETE DEVICES" << endl;
    
    ReportDiscreteDevicesMessage rDiscreteDevices = reportDiscreteDevicesMessageFromJausMessage(msg);
    
    // Freno de estacionamiento
    if(jausByteIsBitSet(rDiscreteDevices->presenceVector,JAUS_DEVICES_PV_PARKING_BIT)){
        if(rDiscreteDevices->parkingBrake)
            cout << "    Freno de mano activado" << endl;
        else
            cout << "    Freno de mano desactivado" << endl;            
    }
    // Marcha
    if(jausByteIsBitSet(rDiscreteDevices->presenceVector,JAUS_DEVICES_PV_GEAR_BIT)){
        cout << "    Marcha: " << rDiscreteDevices->gear << endl;
    }
    reportDiscreteDevicesMessageDestroy(rDiscreteDevices);
    cout << " ---- " << endl;
}

void JausController::fcn_receive_ugv_info(OjCmpt comp, JausMessage msg) {

    cout << "Recibido mensaje UGV INFO - EXP #12" << endl;
    
    UGVInfo12Message rUgvInfo = ugvInfo12MessageFromJausMessage(msg);
    
    // Nivel de bateria
    if(jausByteIsBitSet(rUgvInfo->presenceVector,JAUS_12_PV_BATTERY_LEVEL_BIT)){
        cout << "    Nivel de bateria: " << rUgvInfo->battery_level << endl;
    }
    // Tension de bateria
    if(jausByteIsBitSet(rUgvInfo->presenceVector,JAUS_12_PV_BATTERY_VOLTAGE_BIT)){
        cout << "    Tension de bateria: " << rUgvInfo->battery_voltage << endl;
    }
    // Intensidad de bateria
    if(jausByteIsBitSet(rUgvInfo->presenceVector,JAUS_12_PV_BATTERY_CURRENT_BIT)){
        cout << "    Intensidad de bateria: " << rUgvInfo->battery_current << endl;
    }
    // Temperatura de bateria
    if(jausByteIsBitSet(rUgvInfo->presenceVector,JAUS_12_PV_BATTERY_TEMPERATURE_BIT)){
        cout << "    Temperatura de bateria: " << rUgvInfo->battery_temperature << endl;
    }
    // Temperatura de motor
    if(jausByteIsBitSet(rUgvInfo->presenceVector,JAUS_12_PV_MOTOR_TEMPERATURE_BIT)){
        cout << "    Temperatura de motor: " << rUgvInfo->motor_temperature << endl;
    }
    // RPM de motor
    if(jausByteIsBitSet(rUgvInfo->presenceVector,JAUS_12_PV_MOTOR_RPM_BIT)){
        cout << "    RPM de motor: " << rUgvInfo->motor_rpm << endl;
    }
    // RPM de motor
    if(jausByteIsBitSet(rUgvInfo->presenceVector,JAUS_12_PV_ALARMS_BIT)){
        cout << "    Valor del vector de alarmas: " << rUgvInfo->alarms << endl;
    }
    ugvInfo12MessageDestroy(rUgvInfo);
    cout << " ---- " << endl;
}

// Componente Visual Sensor

void JausController::fcn_receive_report_camera_pose(OjCmpt comp, JausMessage msg) {
    
    cout << "Recibido mensaje REPORT CAMERA POSE" << endl;
    
    ReportCameraPoseMessage rCameraPose = reportCameraPoseMessageFromJausMessage(msg);
    
    // Pan, tilt, zoom
    cout << "    Pan: " << rCameraPose->xCameraAxisDirectionCosineZ << endl;
    cout << "    Tilt: " << rCameraPose->xCameraAxisDirectionCosineY << endl;
    cout << "    Zoom: " << rCameraPose->zCameraOriginMeters << endl;
    
    reportCameraPoseMessageDestroy(rCameraPose);
    cout << " ---- " << endl;
}

void JausController::fcn_receive_report_signaling_elements(OjCmpt comp, JausMessage msg) {

    cout << "Recibido mensaje REPORT SIGNALING ELEMENTS - EXP #25" << endl;
    
    ReportSignalingElements25Message rSignaling = reportSignalingElements25MessageFromJausMessage(msg);
    
    // Luces cortas
    if(rSignaling->dipss)
        cout << "    Luces cortas activadas" << endl;
    else
        cout << "    Luces cortas desactivadas" << endl;
    // Luces de posicion
    if(rSignaling->dipsp)
        cout << "    Luces de posicion activadas" << endl;
    else
        cout << "    Luces de posicion desactivadas" << endl;
    // Luces largas
    if(rSignaling->dipsr)
        cout << "    Luces largas activadas" << endl;
    else
        cout << "    Luces largas desactivadas" << endl;
    // Intermitente derecho
    if(rSignaling->blinker_right)
        cout << "    Intermitente derecho activado" << endl;
    else
        cout << "    Intermitente derecho desactivado" << endl;
    // Intermitente izquierdo
    if(rSignaling->blinker_left)
        cout << "    Intermitente izquierdo activado" << endl;
    else
        cout << "    Intermitente izquierdo desactivado" << endl;
    // Claxon
    if(rSignaling->klaxon)
        cout << "    Claxon activado" << endl;
    else
        cout << "    Claxon desactivado" << endl;
    
    reportSignalingElements25MessageDestroy(rSignaling);
    cout << " ---- " << endl;
}

void JausController::fcn_receive_report_positioner(OjCmpt comp, JausMessage msg) {
    
    cout << "Recibido mensaje REPORT POSITIONER - EXP #20" << endl;
    
    ReportPositioner20Message rPositioner = reportPositioner20MessageFromJausMessage(msg); 
    
    //Pan
    if(jausByteIsBitSet(rPositioner->presenceVector,JAUS_20_PV_PAN_BIT)){
        cout << "    Pan: " << rPositioner->active_pan << endl;
    }
    //Tilt
    if(jausByteIsBitSet(rPositioner->presenceVector,JAUS_20_PV_TILT_BIT)){
        cout << "    Tilt: " << rPositioner->active_tilt << endl;
    }
    reportPositioner20MessageDestroy(rPositioner);
    cout << " ---- " << endl;

}

void JausController::fcn_receive_report_day_time_camera(OjCmpt comp, JausMessage msg) {

    cout << "Recibido mensaje REPORT DAY-TIME CAMERA - EXP #22" << endl;
       
    ReportDayTimeCamera22Message rDayTimeCam = reportDayTimeCamera22MessageFromJausMessage(msg);
    // Zoom
    if(jausByteIsBitSet(rDayTimeCam->presenceVector,JAUS_22_PV_ZOOM_BIT)){
        cout << "    Zoom actual: " << rDayTimeCam->active_zoom << endl;
    }
    // Foco
    if(jausByteIsBitSet(rDayTimeCam->presenceVector,JAUS_22_PV_FOCUS_BIT)){
        cout << "    Foco actual: " << rDayTimeCam->active_focus << endl;
    }
    // Autofoco
    if(jausByteIsBitSet(rDayTimeCam->presenceVector,JAUS_22_PV_AUTOFOCUS_BIT)){
        if(rDayTimeCam->active_autofocus)
            cout << "    Autofoco activado" << endl;
        else
            cout << "    Autofoco desactivado" << endl;
    }
    reportDayTimeCamera22MessageDestroy(rDayTimeCam);
    cout << " ---- " << endl;
}

void JausController::fcn_receive_report_night_time_camera(OjCmpt comp, JausMessage msg) {
    cout << "Recibido mensaje REPORT NIGHT-TIME CAMERA - EXP #24" << endl;
       
    ReportNightTimeCamera24Message rNightTimeCam = reportNightTimeCamera24MessageFromJausMessage(msg);
    // Zoom
    if(jausByteIsBitSet(rNightTimeCam->presenceVector,JAUS_24_PV_ZOOM_BIT)){
        cout << "    Zoom actual: " << rNightTimeCam->active_zoom << endl;
    }
    // Polaridad
    if(jausByteIsBitSet(rNightTimeCam->presenceVector,JAUS_24_PV_POLARITY_BIT)){
        if(rNightTimeCam->active_polarity)
            cout << "    Polaridad activada" << endl;
        else
            cout << "    Polaridad desactivada" << endl;
    }
    reportNightTimeCamera24MessageDestroy(rNightTimeCam);
    cout << " ---- " << endl;
}

// Componente Platform sensor

void JausController::fcn_receive_telemeter_info(OjCmpt comp, JausMessage msg) {
    
    cout << "Recibido mensaje TELEMETER INFO - EXP #10" << endl;
       
    TelemeterInfo10Message rTelemeter = telemeterInfo10MessageFromJausMessage(msg);
    // Ecos
    if(jausByteIsBitSet(rTelemeter->presenceVector,JAUS_10_PV_ECHOES_BIT)){
        for(int i=0;i<5;i++){
            cout << "Eco " << i << ": " << rTelemeter->echoes[i] << endl;
        }
    }
    telemeterInfo10MessageDestroy(rTelemeter);
    cout << " ---- " << endl;
}

// Componente Velocity State Sensor

void JausController::fcn_receive_report_travel_speed(OjCmpt comp, JausMessage msg) {

    cout << "Recibido mensaje REPORT TRAVEL SPEED" << endl;
       
    ReportTravelSpeedMessage rTravelSpd = reportTravelSpeedMessageFromJausMessage(msg);
    cout << "    Velocidad de crucero: " << rTravelSpd->speedMps;
    reportTravelSpeedMessageDestroy(rTravelSpd);
    cout << " ---- " << endl;
}

// Componente Global Pose Sensor

void JausController::fcn_receive_report_global_pose(OjCmpt comp, JausMessage msg) {
    
    cout << "Recibido mensaje REPORT GLOBAL POSE" << endl;
       
    ReportGlobalPoseMessage rGlobalPose = reportGlobalPoseMessageFromJausMessage(msg);
    // Latitud
    if(jausShortIsBitSet(rGlobalPose->presenceVector,JAUS_POSE_PV_LATITUDE_BIT)){
        cout << "    Latitud: " << rGlobalPose->latitudeDegrees << endl;
    }
    // Longitud
    if(jausShortIsBitSet(rGlobalPose->presenceVector,JAUS_POSE_PV_LONGITUDE_BIT)){
        cout << "    Longitud: " << rGlobalPose->longitudeDegrees << endl;
    }
    // Altitud
    if(jausShortIsBitSet(rGlobalPose->presenceVector,JAUS_POSE_PV_ATTITUDE_RMS_BIT)){
        cout << "    Altitud: " << rGlobalPose->attitudeRmsRadians << endl;
    }
    // Roll
    if(jausShortIsBitSet(rGlobalPose->presenceVector,JAUS_POSE_PV_ROLL_BIT)){
        cout << "    Roll: " << rGlobalPose->rollRadians << endl;
    }
    // Pitch
    if(jausShortIsBitSet(rGlobalPose->presenceVector,JAUS_POSE_PV_PITCH_BIT)){
        cout << "    Pitch: " << rGlobalPose->pitchRadians << endl;
    }
    // Yaw
    if(jausShortIsBitSet(rGlobalPose->presenceVector,JAUS_POSE_PV_YAW_BIT)){
        cout << "    Yaw: " << rGlobalPose->yawRadians << endl;
    }
    reportGlobalPoseMessageDestroy(rGlobalPose);
    cout << " ---- " << endl;
}

void JausController::fcn_receive_additional_gpsins_info(OjCmpt comp, JausMessage msg) {

    cout << "Recibido mensaje ADDITIONAL GPS/INS INFO - EXP #4" << endl;
    
    AditionalGPSINSInfo4Message rAdGPSInfo = aditionalGPSINSInfo4MessageFromJausMessage(msg);
    // Aceleracion longitudinal
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_LONGITUDINAL_ACC_BIT)){
        cout << "    Aceleracion longitudinal: " << rAdGPSInfo->longitudinal_acc << endl;
    }
    // Aceleracion lateral
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_LATERAL_ACC_BIT)){
        cout << "    Aceleracion lateral: " << rAdGPSInfo->lateral_acc << endl;
    }
    // Aceleracion vertical
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_VERTICAL_ACC_BIT)){
        cout << "    Aceleracion vertical: " << rAdGPSInfo->vertical_acc << endl;
    }
    // Estado GPS/INS
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_GPSINS_STATUS_BIT)){
        cout << "    Valor de estado de GPS: " << rAdGPSInfo->gpsins_status << endl;
    }
    // Calidad de la medida
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_MEASURE_QUALITY_BIT)){
        for(int i=0 ; i<3 ; i++){
            cout << "   q" << i << ": " << rAdGPSInfo->measure_quality[i] << endl;
        }
    }
    // Desviacion estandar latitud
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_ST_LAT_DEVIATION_BIT)){
        cout << "    Desviacion estandar latitud: " << rAdGPSInfo->st_lat_deviation << endl;
    }
    // Desviacion estandar longitud
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_ST_LON_DEVIATION_BIT)){
        cout << "    Desviacion estandar longitud: " << rAdGPSInfo->st_lon_deviation << endl;
    }
    // Desviacion estandar altitud
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_ST_ALT_DEVIATION_BIT)){
        cout << "    Desviacion estandar altitud: " << rAdGPSInfo->st_alt_deviation << endl;
    }
    // Correcciones DGPS
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_DGPS_CORRECTIONS_BIT)){
        if(rAdGPSInfo->dgps_corrections)
            cout << "    Correcciones DGPS: true" << endl;
        else
            cout << "    Correcciones DGPS: false" << endl;
    }
    // Disponibilidad de GPS
    if(jausShortIsBitSet(rAdGPSInfo->presenceVector,JAUS_4_PV_GPSINS_AVAILABILITY_BIT)){
        if(rAdGPSInfo->gpsins_availability)
            cout << "    Disponibilidad GPS: true" << endl;
        else
            cout << "    Disponibilidad GPS: false" << endl;
    }
    aditionalGPSINSInfo4MessageDestroy(rAdGPSInfo);
    cout << " ---- " << endl;
}

// Componente HeartBeat Information

void JausController::fcn_receive_heartbeat_channel_state(OjCmpt comp, JausMessage msg) {
    
    cout << "Recibido mensaje HEARTBEAT - CHANNEL STATE - EXP #16" << endl;
    
    HeartbeatChannelState16Message rHeartBeatChannel = heartbeatChannelState16MessageFromJausMessage(msg);
    
    // Identificador de nodo
    if(jausByteIsBitSet(rHeartBeatChannel->presenceVector,JAUS_16_PV_NODE_ID_BIT)){
        cout << "    ID Nodo: " << rHeartBeatChannel->node_id << endl;
    }
    // Estado del canal primario
    if(jausByteIsBitSet(rHeartBeatChannel->presenceVector,JAUS_16_PV_PRIMARY_CHANNEL_STATUS_BIT)){
        cout << "    Estado del canal primario: " << rHeartBeatChannel->primary_channel_status << endl;
    }
    // Estado del canal backup
    if(jausByteIsBitSet(rHeartBeatChannel->presenceVector,JAUS_16_PV_BACKUP_CHANNEL_STATUS_BIT)){
        cout << "    Estado del canal backup: " << rHeartBeatChannel->backup_channel_status << endl;
    }
    // SNR del canal primario
    if(jausByteIsBitSet(rHeartBeatChannel->presenceVector,JAUS_16_PV_PRIMARY_CHANNEL_SNR_BIT)){
        cout << "    SNR del canal primario: " << rHeartBeatChannel->primary_channel_snr << endl;
    }
    // SNR del canal backup
    if(jausByteIsBitSet(rHeartBeatChannel->presenceVector,JAUS_16_PV_BACKUP_CHANNEL_SNR_BIT)){
        cout << "    SNR del canal backup: " << rHeartBeatChannel->backup_channel_snr << endl;
    }
    heartbeatChannelState16MessageDestroy(rHeartBeatChannel);
    cout << " ---- " << endl;
}

void JausController::fcn_receive_heartbeat_position_info(OjCmpt comp, JausMessage msg) {
    
    cout << "Recibido mensaje HEARTBEAT - POSITION INFO - EXP #17" << endl;
    
    HeartbeatPositionInfo17Message rHeartBeatPos = heartbeatPositionInfo17MessageFromJausMessage(msg);
    
    // Latitud
    if(jausByteIsBitSet(rHeartBeatPos->presenceVector,JAUS_17_PV_LATITUDE_BIT)){
        cout << "    Latitud: " << rHeartBeatPos->latitude << endl;
    }
    // Longitud
    if(jausByteIsBitSet(rHeartBeatPos->presenceVector,JAUS_17_PV_LONGITUDE_BIT)){
        cout << "    Longitud: " << rHeartBeatPos->longitude << endl;
    }
    // Altitud
    if(jausByteIsBitSet(rHeartBeatPos->presenceVector,JAUS_17_PV_ALTITUDE_BIT)){
        cout << "    Altitud: " << rHeartBeatPos->altitude << endl;
    }
    // Heading
    if(jausByteIsBitSet(rHeartBeatPos->presenceVector,JAUS_17_PV_HEADING_BIT)){
        cout << "    Heading: " << rHeartBeatPos->heading << endl;
    }
    // Velocidad 
    if(jausByteIsBitSet(rHeartBeatPos->presenceVector,JAUS_17_PV_SPEED_BIT)){
        cout << "    Velocidad: " << rHeartBeatPos->speed << endl;
    }
    heartbeatPositionInfo17MessageDestroy(rHeartBeatPos);
    cout << " ---- " << endl;
}