
#include "JausController.h"

// Declaracion para patron Singleton
bool JausController::instanceCreated = false;
JausController *JausController::instance = NULL;

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
        
        ROS_INFO("[Control] Communications - No se ha podido inicializar JAUS");
        
    }
    
    /*
     * Creacion de componentes
     * 
     */
    
    // Mission Spooler
    missionSpoolerComponent = ojCmptCreate((char *) "Mission Spooler", JAUS_MISSION_SPOOLER, 1);
    if (missionSpoolerComponent == NULL) {
        ROS_INFO("[Control] Communication - No se ha podido crear el componente MISSION SPOOLER");
        exit(0);
    }else{
        
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(missionSpoolerComponent, JAUS_MISSION_SPOOLER, JAUS_REPORT_MISSION_STATUS, 0xFF);
        
        // Mensajes que recibe
        ojCmptAddServiceInputMessage(missionSpoolerComponent, JAUS_MISSION_SPOOLER, JAUS_RUN_MISSION, 0xFF);

        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(missionSpoolerComponent, JAUS_RUN_MISSION, fcn_receive_run_mission);
        
    }
    
    // Primitive Driver
    primitiveDriverComponent = ojCmptCreate((char *) "Primitive Driver", JAUS_PRIMITIVE_DRIVER, 1);
    if (primitiveDriverComponent == NULL) {
        ROS_INFO("[Control] Communication - No se ha podido crear el componente PRIMITIVE DRIVER");
        exit(0);
    }else{
    
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_WRENCH_EFFORT, 0xFF);
        ojCmptAddServiceOutputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_DISCRETE_DEVICES, 0xFF);
        ojCmptAddServiceOutputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_UGV_INFO_12, 0xFF);
        
        // Mensajes que recibe
        ojCmptAddServiceInputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_SET_WRENCH_EFFORT, 0xFF);
        ojCmptAddServiceInputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_SET_DISCRETE_DEVICES, 0xFF);
        
        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(primitiveDriverComponent, JAUS_SET_WRENCH_EFFORT, fcn_receive_set_wrench_effort);
        ojCmptSetMessageCallback(primitiveDriverComponent, JAUS_SET_DISCRETE_DEVICES, fcn_receive_set_discrete_devices);
    }
    
    // Visual Sensor
    visualSensorComponent = ojCmptCreate((char *) "Visual Sensor", JAUS_VISUAL_SENSOR, 1);
    if (visualSensorComponent == NULL) {
        ROS_INFO("[Control] Communication - No se ha podido crear el componente VISUAL SENSOR");
        exit(0);
    }else{
        
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_CAMERA_POSE, 0xFF);
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_SIGNALING_ELEMENTS_25, 0xFF);
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_POSITIONER_20, 0xFF);
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_DAY_TIME_CAMERA_22, 0xFF);
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_NIGHT_TIME_CAMERA_24, 0xFF);
        
        // Mensajes que recibe
        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_CAMERA_POSE, 0xFF);
        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_SIGNALING_ELEMENTS_18, 0xFF);
        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_POSITIONER_19, 0xFF);
        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_DAY_TIME_CAMERA_21, 0xFF);
        ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_NIGHT_TIME_CAMERA_23, 0xFF);
        
        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_SET_CAMERA_POSE, fcn_receive_set_camera_pose);
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_SET_SIGNALING_ELEMENTS_18, fcn_receive_set_signaling_elements);
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_SET_POSITIONER_19, fcn_receive_set_positioner);
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_SET_DAY_TIME_CAMERA_21, fcn_receive_set_day_time_camera);
        ojCmptSetMessageCallback(visualSensorComponent, JAUS_SET_NIGHT_TIME_CAMERA_23, fcn_receive_set_night_time_camera);
    
    }
    
    // Platform Sensor
    platformSensorComponent = ojCmptCreate((char *) "Platform Sensor", JAUS_PLATFORM_SENSOR, 1);
    if (platformSensorComponent == NULL) {
        ROS_INFO("[Control] Communication - No se ha podido crear el componente PLATFORM SENSOR");
        exit(0);
    }else{
        
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(platformSensorComponent, JAUS_PLATFORM_SENSOR, JAUS_TELEMETER_INFO_10, 0xFF);
        
        // Mensajes que recibe
        ojCmptAddServiceInputMessage(platformSensorComponent, JAUS_PLATFORM_SENSOR, JAUS_TELEMETER_INFO_10, 0xFF);
        
        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(platformSensorComponent, JAUS_TELEMETER_INFO_10, fcn_receive_telemeter_info);
        
    }
    
    // Global Waypoint Driver
    globalWaypointDriverComponent = ojCmptCreate((char *)"Global Waypoint Driver", JAUS_GLOBAL_WAYPOINT_DRIVER,1);
    if (globalWaypointDriverComponent == NULL) {
        ROS_INFO("[Control] Communication - No se ha podido crear el componente GLOBAL WAYPOINT DRIVER");
        exit(0);
    }else{
        
        // REVISAR COMPONENTE INNECESARIO PARA UGV
        // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    }
    
    // Velocity State Sensor
    velocityStateSensorComponent = ojCmptCreate((char *)"Velocity State Sensor",JAUS_VELOCITY_STATE_SENSOR,1);
    if (velocityStateSensorComponent == NULL) {
        ROS_INFO("[Control] Communication - No se ha podido crear el componente VELOCITY STATE SENSOR");
        exit(0);
    }else{
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_REPORT_TRAVEL_SPEED, 0xFF);
        ojCmptAddServiceOutputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_REPORT_VELOCITY_STATE, 0xFF);
        
        // Mensajes que recibe
        ojCmptAddServiceInputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_SET_TRAVEL_SPEED, 0xFF);

        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(velocityStateSensorComponent, JAUS_SET_TRAVEL_SPEED, fcn_receive_set_travel_speed);
    
    }
    
    // Global Pose Sensor
    globalPoseSensorComponent = ojCmptCreate((char *)"Global Pose Sensor",JAUS_GLOBAL_POSE_SENSOR,1);
    if (globalPoseSensorComponent == NULL) {
        ROS_INFO("[Control] Communication - No se ha podido crear el componente GLOBAL POSE SENSOR");
        exit(0);
    }else{
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(globalPoseSensorComponent, JAUS_GLOBAL_POSE_SENSOR, JAUS_REPORT_GLOBAL_POSE, 0xFF);
        ojCmptAddServiceOutputMessage(globalPoseSensorComponent, JAUS_GLOBAL_POSE_SENSOR, JAUS_ADITIONAL_GPSINS_INFO_4, 0xFF);
        
    }
    
    // HeartBeat Information
    heartBeatInformationComponent = ojCmptCreate((char *)"HeartBeat Information", JAUS_HEARTBEAT_INFORMATION,1);
    if (heartBeatInformationComponent == NULL) {
        ROS_INFO("[Control] Communication - No se ha podido crear el componente HEARTBEAT INFORMATION");
        exit(0);
    }else{
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(heartBeatInformationComponent, JAUS_HEARTBEAT_INFORMATION, JAUS_HEARTBEAT_CHANNEL_STATE_16, 0xFF);
        ojCmptAddServiceOutputMessage(heartBeatInformationComponent, JAUS_HEARTBEAT_INFORMATION, JAUS_HEARTBEAT_POSITION_INFO_17, 0xFF);
        
        // Mensajes que recibe
        ojCmptAddServiceInputMessage(heartBeatInformationComponent, JAUS_HEARTBEAT_INFORMATION, JAUS_HEARTBEAT_CHANNEL_STATE_16, 0xFF);
        ojCmptAddServiceInputMessage(heartBeatInformationComponent, JAUS_HEARTBEAT_INFORMATION, JAUS_HEARTBEAT_POSITION_INFO_17, 0xFF);

        // Funciones de recepcion de mensajes (Callbacks)
        ojCmptSetMessageCallback(heartBeatInformationComponent, JAUS_HEARTBEAT_CHANNEL_STATE_16, fcn_receive_heartbeat_channel_state);
        ojCmptSetMessageCallback(heartBeatInformationComponent, JAUS_HEARTBEAT_POSITION_INFO_17, fcn_receive_heartbeat_position_info);
    
    }
}

/* 
 * INFORMACION DE MODO DE OPERACION
 * CORRESPONDENCIA JAUS: REPORT MISSION STATUS
 */

void JausController::informStatus() {
    
    // Obtencion del estado
    int status;
    ros::NodeHandle nh;
    nh.getParam("vehicleStatus",status);
    JausMessage jMsg = NULL;
    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = instance->subsystemController;
    jAdd->node = instance->nodeController;
    jAdd->component = JAUS_MISSION_SPOOLER;

    // Generacion de mensaje especifico UGV Info
    ReportMissionStatusMessage rmsm = reportMissionStatusMessageCreate();
    rmsm->missionId = status;
    jausAddressCopy(rmsm->destination, jAdd);
    
    // Generacion de mensaje JUAS global
    jMsg = reportMissionStatusMessageToJausMessage(rmsm);
    
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->missionSpoolerComponent, jMsg);
    } else {
        ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion electrica de vehiculo");
    }
    // Liberacion de memoria
    reportMissionStatusMessageDestroy(rmsm);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
}

/*******************************************************************************
 *******************************************************************************
 *                              CALLBACKS JAUS                                 *
 *******************************************************************************
 ******************************************************************************/

// Componente Mission Spooler

void JausController::fcn_receive_run_mission(OjCmpt cmp, JausMessage msg) {
    RunMissionMessage rMission = runMissionMessageFromJausMessage(msg);
    CITIUS_Control_Communication::srv_vehicleStatus vehicleStatus;
    
    // Desde comunicaciones no se puede cambiar a otro modo que no sea conduccion
    // u observacion
    
    short tempStatus = rMission->missionId;
    
    switch(tempStatus){
        
        case 5: // Conduccion (Valor de ICD)
            vehicleStatus.request.status = OPERATION_MODE_CONDUCCION;
            instance->clientStatus.call(vehicleStatus);
            instance->informStatus();
            break;
            
        case 6: // Observacion (Valor de ICD)
            vehicleStatus.request.status = OPERATION_MODE_OBSERVACION;
            instance->clientStatus.call(vehicleStatus);
            instance->informStatus();
            break;
            
        default:
            break;
    }

    // Liberacion de memoria
    runMissionMessageDestroy(rMission);
}

// Componente Primitive Driver

void JausController::fcn_receive_set_wrench_effort(OjCmpt cmp, JausMessage msg) {
    SetWrenchEffortMessage sWrenchEffort = setWrenchEffortMessageFromJausMessage(msg);
    CITIUS_Control_Communication::msg_command command;
    
    // Comprobacion de direccion
    if((sWrenchEffort->presenceVector & PRESENCE_VECTOR_STEER) == PRESENCE_VECTOR_STEER){
        command.id_device = STEERING;
        command.value = sWrenchEffort->propulsiveRotationalEffortZPercent;
        //pubCommand.publish(command);
        instance->pubCommand.publish(command);
    }
    
    // Comprobacion de acelerador
    if((sWrenchEffort->presenceVector & PRESENCE_VECTOR_THROTTLE) == PRESENCE_VECTOR_THROTTLE){
        command.id_device = THROTTLE;
        command.value = sWrenchEffort->propulsiveLinearEffortXPercent;
        instance->pubCommand.publish(command);
    }
    
    // Comprobacion de freno de servicio
    if((sWrenchEffort->presenceVector & PRESENCE_VECTOR_BRAKE) == PRESENCE_VECTOR_BRAKE){
        command.id_device = BRAKE;
        command.value = sWrenchEffort->resistiveLinearEffortXPercent;
        instance->pubCommand.publish(command);
    }
    
    // Liberacion de memoria
    setWrenchEffortMessageDestroy(sWrenchEffort);
}

void JausController::fcn_receive_set_discrete_devices(OjCmpt cmp, JausMessage msg) {
    SetDiscreteDevicesMessage sDiscreteDevice = setDiscreteDevicesMessageFromJausMessage(msg);
    CITIUS_Control_Communication::msg_command command;
    
    // Comprobacion de freno de estacionamiento
    if((sDiscreteDevice->presenceVector & PRESENCE_VECTOR_PARKING_BRAKE)==PRESENCE_VECTOR_PARKING_BRAKE){
        command.id_device = HANDBRAKE;
        command.value = sDiscreteDevice->parkingBrake;
        instance->pubCommand.publish(command);
    }
    
    // Comprobacion de marcha
    if((sDiscreteDevice->presenceVector & PRESENCE_VECTOR_GEAR)==PRESENCE_VECTOR_GEAR){
        command.id_device = GEAR;
        command.value = sDiscreteDevice->gear;
        instance->pubCommand.publish(command);
    }

    // Comprobacion arranque/parada de motor
    // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    // Liberacion de memoria
    setDiscreteDevicesMessageDestroy(sDiscreteDevice);

}

// Componente Visual Sensor

void JausController::fcn_receive_set_camera_pose(OjCmpt cmp, JausMessage msg) {
    SetCameraPoseMessage sCameraPose = setCameraPoseMessageFromJausMessage(msg);
    // Camara delantera
    if(sCameraPose->cameraID == FRONT_CAMERA_ID){
        
        CITIUS_Control_Communication::msg_ctrlFrontCamera fcCommand;
        fcCommand.isPan = false;
        fcCommand.isTilt = false;
        fcCommand.isZoom = false;
        
        // Comprobacion de pan
        if((sCameraPose->presenceVector & PRESENCE_VECTOR_PAN)==PRESENCE_VECTOR_PAN){
            fcCommand.isPan = true;
            fcCommand.pan = sCameraPose->zAngularPositionOrRatePercent;
        }
        
        // Comprobacion de tilt
        if((sCameraPose->presenceVector & PRESENCE_VECTOR_TILT)==PRESENCE_VECTOR_TILT){
            fcCommand.isTilt = true;
            fcCommand.tilt = sCameraPose->yAngularPositionOrRatePercent;
        }
        
        // Comprobacion de zoom
        if((sCameraPose->presenceVector & PRESENCE_VECTOR_ZOOM)==PRESENCE_VECTOR_ZOOM){
            fcCommand.isZoom = true;
            fcCommand.zoom = sCameraPose->zLinearPositionOrRatePercent;
        }
        
        instance->pubCtrlFrontCamera.publish(fcCommand);
        
    }
    // Camara trasera
    else if(sCameraPose->cameraID == REAR_CAMERA_ID){
    
        CITIUS_Control_Communication::msg_ctrlRearCamera rcCommand;
        rcCommand.isPan = false;
        rcCommand.isTilt = false;
        rcCommand.isZoom = false;
        
        // Comprobacion de pan
        if((sCameraPose->presenceVector & PRESENCE_VECTOR_PAN)==PRESENCE_VECTOR_PAN){
            rcCommand.isPan = true;
            rcCommand.pan = sCameraPose->zAngularPositionOrRatePercent;
        }
        
        // Comprobacion de tilt
        if((sCameraPose->presenceVector & PRESENCE_VECTOR_TILT)==PRESENCE_VECTOR_TILT){
            rcCommand.isTilt = true;
            rcCommand.tilt = sCameraPose->yAngularPositionOrRatePercent;
        }
        
        // Comprobacion de zoom
        if((sCameraPose->presenceVector & PRESENCE_VECTOR_ZOOM)==PRESENCE_VECTOR_ZOOM){
            rcCommand.isZoom = true;
            rcCommand.zoom = sCameraPose->zLinearPositionOrRatePercent;
        }
        
        instance->pubCtrlRearCamera.publish(rcCommand);
        
    }
    
    // Liberacion de memoria
    setCameraPoseMessageDestroy(sCameraPose);
}

void JausController::fcn_receive_set_signaling_elements(OjCmpt cmp, JausMessage msg) {
    SetSignalingElements18Message sSignaling = setSignalingElements18MessageFromJausMessage(msg);
    CITIUS_Control_Communication::msg_command command;
    
    // Comprobacion de luces de posicion
    if((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSP) == PRESENCE_VECTOR_DIPSP){
        command.id_device = DIPSP;
        command.value = sSignaling->dipsp;
        instance->pubCommand.publish(command);
    }
    
    // Comprobacion de luces cortas
    if((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSS) == PRESENCE_VECTOR_DIPSS){
        command.id_device = DIPSS;
        command.value = sSignaling->dipss;
        instance->pubCommand.publish(command);
    }
    
    // Comprobacion de luces largas
    if((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSR) == PRESENCE_VECTOR_DIPSR){
        command.id_device = DIPSR;
        command.value = sSignaling->dipsr;
        instance->pubCommand.publish(command);
    }
    
    // Comprobacion de intermitente derecha
    if((sSignaling->presenceVector & PRESENCE_VECTOR_BLINKER_RIGHT) == PRESENCE_VECTOR_BLINKER_RIGHT){
        command.id_device = BLINKER_RIGHT;
        command.value = sSignaling->blinker_right;
        instance->pubCommand.publish(command);
    }
    
    // Comprobacion de intermitente izquierda
    if((sSignaling->presenceVector & PRESENCE_VECTOR_BLINKER_LEFT) == PRESENCE_VECTOR_BLINKER_LEFT){
        command.id_device = BLINKER_LEFT;
        command.value = sSignaling->blinker_left;
        instance->pubCommand.publish(command);
    }
    
    // Comprobacion de claxon
    if((sSignaling->presenceVector & PRESENCE_VECTOR_KLAXON) == PRESENCE_VECTOR_KLAXON){
        command.id_device = KLAXON;
        command.value = sSignaling->klaxon;
        instance->pubCommand.publish(command);
    }
    
    // Liberacion de memoria
    setSignalingElements18MessageDestroy(sSignaling);
}

void JausController::fcn_receive_set_positioner(OjCmpt cmp, JausMessage msg) {
    SetPositioner19Message setPositioner = setPositioner19MessageFromJausMessage(msg);
    
    if(jausByteIsBitSet(setPositioner->presenceVector, JAUS_19_PV_PAN_BIT)){
        CITIUS_Control_Communication::srv_panAbsolutePosition serviceAbsPan;
        serviceAbsPan.request.panPosition = setPositioner->pan;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientPosPanAbs.call(serviceAbsPan)){
                numOfAttemps = 10;
                if(serviceAbsPan.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de Pan absoluto a Positioner");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de Pan absoluto a Positioner");
        }
    }
    
    if(jausByteIsBitSet(setPositioner->presenceVector, JAUS_19_PV_SPIN_VELOCITY_BIT)){
        CITIUS_Control_Communication::srv_panRate servicePanRate;
        servicePanRate.request.panRate = setPositioner->spin_velocity;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientPosPanRate.call(servicePanRate)){
                numOfAttemps = 10;
                if(servicePanRate.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de Vel. Pan absoluto a Positioner");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de Vel. Pan a Positioner");
        }
    }
    
    if(jausByteIsBitSet(setPositioner->presenceVector, JAUS_19_PV_TILT_BIT)){
        CITIUS_Control_Communication::srv_tiltAbsolutePosition serviceAbsTilt;
        serviceAbsTilt.request.tiltPosition = setPositioner->tilt;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientPosTiltAbs.call(serviceAbsTilt)){
                numOfAttemps = 10;
                if(serviceAbsTilt.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de Tilt absoluto a Positioner");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de Tilt absoluto a Positioner");
        }
    }
    
    if(jausByteIsBitSet(setPositioner->presenceVector, JAUS_19_PV_ELEVATION_VELOCITY_BIT)){
        CITIUS_Control_Communication::srv_tiltRate serviceTiltRate;
        serviceTiltRate.request.tiltRate = setPositioner->elevation_velocity;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientPosTiltRate.call(serviceTiltRate)){
                numOfAttemps = 10;
                if(serviceTiltRate.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de Tilt Rate a Positioner");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de Tilt Rate a Positioner");
        }
    }
    
}

void JausController::fcn_receive_set_day_time_camera(OjCmpt cmp, JausMessage msg) {
    SetDayTimeCamera21Message setDayCam = setDayTimeCamera21MessageFromJausMessage(msg);
    
    if(jausByteIsBitSet(setDayCam->presenceVector,JAUS_21_PV_DIRECT_ZOOM_BIT)){
        CITIUS_Control_Communication::srv_zoomDirect serviceDirectZoom;    
        serviceDirectZoom.request.zoomDirect = setDayCam->direct_zoom;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientTVCameraDirectZoom.call(serviceDirectZoom)){
                numOfAttemps = 10;
                if(serviceDirectZoom.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de Direct Zoom a TV Camera");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de Direct Zoom a TV Camera");
        }
    }
    
    if(jausByteIsBitSet(setDayCam->presenceVector,JAUS_21_PV_CONTINUOUS_ZOOM_BIT)){
        CITIUS_Control_Communication::srv_zoomCommand serviceContZoom;
        serviceContZoom.request.zoomCommand = setDayCam->continuous_zoom;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientTVCameraContZoom.call(serviceContZoom)){
                numOfAttemps = 10;
                if(serviceContZoom.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de Cont. Zoom a TV Camera");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de Cont. Zoom a TV Camera");
        }
    }
    
    if(jausByteIsBitSet(setDayCam->presenceVector,JAUS_21_PV_FOCUS_BIT)){
        CITIUS_Control_Communication::srv_focusDirect serviceDirectFocus;
        serviceDirectFocus.request.focusDirect = setDayCam->focus;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientTVCameraFocus.call(serviceDirectFocus)){
                numOfAttemps = 10;
                if(serviceDirectFocus.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de Focus a TV Camera");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de Focus a TV Camera");
        }
    }
    
    if(jausByteIsBitSet(setDayCam->presenceVector,JAUS_21_PV_AUTOFOCUS_BIT)){
        CITIUS_Control_Communication::srv_autofocusMode serviceAutofocus;
        serviceAutofocus.request.autofocus = setDayCam->autofocus;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientTVCameraAutofocus.call(serviceAutofocus)){
                numOfAttemps = 10;
                if(serviceAutofocus.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de AutoFocus a TV Camera");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de AutoFocus a TV Camera");
        }
    }
}

void JausController::fcn_receive_set_night_time_camera(OjCmpt cmp, JausMessage msg) {
    SetNightTimeCamera23Message setNightCam = setNightTimeCamera23MessageFromJausMessage(msg);
    
    if(jausByteIsBitSet(setNightCam->presenceVector,JAUS_23_PV_ZOOM_BIT)){
        CITIUS_Control_Communication::srv_dzoom serviceZoom;
        serviceZoom.request.newZoom = setNightCam->zoom;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientIRCameraZoom.call(serviceZoom)){
                numOfAttemps = 10;
                if(serviceZoom.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de Zoom a IR Camera");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de Zoom a IR Camera");
        }
    }
    
    if(jausByteIsBitSet(setNightCam->presenceVector,JAUS_23_PV_POLARITY_BIT)){
        CITIUS_Control_Communication::srv_polarity servicePolarity;
        servicePolarity.request.newPolarity = setNightCam->polarity;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientIRCameraPolarity.call(servicePolarity)){
                numOfAttemps = 10;
                if(servicePolarity.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Req. de Polarity a IR Camera");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Req. de Polarity a IR Camera");
        }
    }
    
}

// Componente Platform Sensor

void JausController::fcn_receive_telemeter_info(OjCmpt cmp, JausMessage msg){
    TelemeterInfo10Message telemeterMsg = telemeterInfo10MessageFromJausMessage(msg);
    CITIUS_Control_Communication::srv_shoot serviceTelemeter;
    
    if(jausByteIsBitSet(telemeterMsg->presenceVector,JAUS_10_PV_SHOOT_BIT)){
        CITIUS_Control_Communication::srv_shoot serviceShootTelemeter;
        short numOfAttemps = 0;
        while(numOfAttemps < 5){
            if(instance->clientShootTel.call(serviceShootTelemeter)){
                numOfAttemps = 10;
                if(serviceTelemeter.response.ret == false)
                    ROS_INFO("[Control] Communications - Error en el Shoot del telemetro");
            }else{
                numOfAttemps++;
            }
        }
        if(numOfAttemps == 5){
            ROS_INFO("[Control] Communications - Error en el Shoot del telemetro");
        }
    }
}

// Componente Velocity State Sensor

void JausController::fcn_receive_set_travel_speed(OjCmpt cmp, JausMessage msg) {
    SetTravelSpeedMessage sTravelSpeed = setTravelSpeedMessageFromJausMessage(msg);
    CITIUS_Control_Communication::msg_command command;
    
    // No hay comprobacion de presence vector. Un solo parametro
    command.id_device = CRUISING_SPEED;
    command.value = sTravelSpeed->speedMps;
    instance->pubCommand.publish(command);
    
    // Liberacion de memoria
    setTravelSpeedMessageDestroy(sTravelSpeed);
}

// Componente HeartBeat Information

void JausController::fcn_receive_heartbeat_channel_state(OjCmpt cmp, JausMessage msg) {

}

void JausController::fcn_receive_heartbeat_position_info(OjCmpt cmp, JausMessage msg) {

}

