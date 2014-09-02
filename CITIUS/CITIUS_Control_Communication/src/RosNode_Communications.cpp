
#include "RosNode_Communications.h"

RosNode_Communications::RosNode_Communications() {

}

/*******************************************************************************
 *******************************************************************************
 *                     INICIALIZACION DE ARTEFACTOS ROS                        *
 *******************************************************************************
 ******************************************************************************/
void RosNode_Communications::initROS() {
    ros::NodeHandle nh;
    // Inicializacion de publicadores
    this->pubCommand = nh.advertise<CITIUS_Control_Communication::msg_command>("command", 1000);
    this->pubCtrlFrontCamera = nh.advertise<CITIUS_Control_Communication::msg_ctrlFrontCamera>("ctrlFrontCamera", 1000);
    this->pubCtrlRearCamera = nh.advertise<CITIUS_Control_Communication::msg_ctrlRearCamera>("ctrlRearCamera", 1000);
    // Inicializacion de suscriptores
    this->subsElectricInfo = nh.subscribe("electricInfo", 1000, &RosNode_Communications::fnc_subs_electricInfo, this);
    this->subsVehicleInfo = nh.subscribe("vehicleInfo", 1000, &RosNode_Communications::fnc_subs_vehicleInfo, this);
    this->subsFrontCameraInfo = nh.subscribe("frontCameraInfo", 1000, &RosNode_Communications::fnc_subs_frontCameraInfo, this);
    this->subsRearCameraInfo = nh.subscribe("rearCameraInfo", 1000, &RosNode_Communications::fnc_subs_rearCameraInfo, this);
    this->subsPosOriInfo = nh.subscribe("posOriInfo", 1000, &RosNode_Communications::fnc_subs_posOriInfo, this);
    this->subsIRCameraInfo = nh.subscribe("IRInformation", 1000, &RosNode_Communications::fcn_subs_irCameraInfo, this);
    this->subsTelemeterInfo = nh.subscribe("LRFEchoesFound", 1000, &RosNode_Communications::fcn_subs_positionerInfo, this);
    this->subsTVCameraInfo = nh.subscribe("TVInformation", 1000, &RosNode_Communications::fcn_subs_tvCameraInfo, this);
    this->subsPositionerInfo = nh.subscribe("PanTiltPosition", 1000, &RosNode_Communications::fcn_subs_telemeterInfo, this);
    // Inicializacion de servidores
    this->clientStatus = nh.serviceClient<CITIUS_Control_Communication::srv_vehicleStatus>("nodeStateDriving");
}

/*******************************************************************************
 *******************************************************************************
 *                     INICIALIZACION DE ARTEFACTOS JAUS                        *
 *******************************************************************************
 ******************************************************************************/

void RosNode_Communications::initJAUS() {
    
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

/*******************************************************************************
 *******************************************************************************
 *                     GETTER AND SETTER DE ATRIBUTOS                          *
 *******************************************************************************
 ******************************************************************************/
ros::Publisher RosNode_Communications::getPublisherFrontCamera() {
    return this->pubCtrlFrontCamera;
}

ros::Publisher RosNode_Communications::getPublisherRearCamera() {
    return this->pubCtrlRearCamera;
}

ros::Publisher RosNode_Communications::getPublisherCommand() {
    return this->pubCommand;
}

/*******************************************************************************
 *******************************************************************************
 *                               CALLBACKS ROS                                 *
 *******************************************************************************
 ******************************************************************************/

/* 
 * INFORMACION DE CAMARA DELANTERA
 * CORRESPONDENCIA JAUS: REPORT CAMERA POSE
 */
void RosNode_Communications::fnc_subs_frontCameraInfo(CITIUS_Control_Communication::msg_frontCameraInfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion de camara delantera");
    
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromCameraInfo(this->subsystemController,this->nodeController,FRONT_CAMERA_ID,msg.pan,msg.tilt,msg.zoom);
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(this->visualSensorComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    }else{
        ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de camara delantera");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION DE CAMARA TRASERA
 * CORRESPONDENCIA JAUS: REPORT CAMERA POSE
 */
void RosNode_Communications::fnc_subs_rearCameraInfo(CITIUS_Control_Communication::msg_rearCameraInfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion de camara trasera");
    
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromCameraInfo(this->subsystemController,this->nodeController,REAR_CAMERA_ID,msg.pan,msg.tilt,msg.zoom);
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(this->visualSensorComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    }else{
        ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de camara trasera");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION DE VEHICULO
 * CORRESPONDENCIA JAUS: REPORT DISCRETE DEVICE / REPORT WRENCH EFFORT / REPORT SIGNALING ELEMENTS
 */
void RosNode_Communications::fnc_subs_vehicleInfo(CITIUS_Control_Communication::msg_vehicleInfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion de vehiculo");
    
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromVehicleInfo(this->subsystemController,this->nodeController,msg.id_device,msg.value);
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    }else{
        ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de vehiculo");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION ELECTRICA
 * CORRESPONDENCIA JAUS: UGV INFO
 */
void RosNode_Communications::fnc_subs_electricInfo(CITIUS_Control_Communication::msg_electricInfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion electrica");
    
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromElectricInfo(this->subsystemController,this->nodeController,msg.id_device,msg.value);
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    }else{
        ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion electrica de vehiculo");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION DE POSICION / ORIENTACION
 * CORRESPONDENCIA JAUS: REPORT GLOBAL POSE / REPORT VELOCITY STATE / ADDITIONAL GPS/INS INFO
 */
void RosNode_Communications::fnc_subs_posOriInfo(CITIUS_Control_Communication::msg_posOriInfo msg) {

    ROS_INFO("[Control] Communications - Recibido mensaje de Position Orientation");
    
    JausMessage jMsg = NULL;
    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_GLOBAL_POSE_SENSOR;
    
    // MENSAJE REPORT GLOBAL POSE
    // Formacion del mensaje
    ReportGlobalPoseMessage rgpm = reportGlobalPoseMessageCreate();
    rgpm->presenceVector = 0x0077;
    rgpm->latitudeDegrees = msg.latitude;
    rgpm->longitudeDegrees = msg.longitude;
    rgpm->attitudeRmsRadians = msg.altitude;
    rgpm->rollRadians = msg.roll;
    rgpm->pitchRadians = msg.pitch;
    rgpm->yawRadians = msg.yaw;
    
    jausAddressCopy(rgpm->destination, jAdd);
    jMsg = reportGlobalPoseMessageToJausMessage(rgpm);
    reportGlobalPoseMessageDestroy(rgpm);
    
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(globalPoseSensorComponent, jMsg);
    }
    
    // MENSAJE REPORT VELOCITY STATE
    // Formacion del mensaje
    ReportVelocityStateMessage rvsm = reportVelocityStateMessageCreate();
    rvsm->presenceVector = 0x0007;
    rvsm->velocityXMps = msg.velX;
    rvsm->velocityYMps = msg.velY;
    rvsm->velocityZMps = msg.velZ;
    
    // Direccion
    jAdd->component = JAUS_VELOCITY_STATE_SENSOR;
    
    jausAddressCopy(rvsm->destination, jAdd);
    jMsg = reportVelocityStateMessageToJausMessage(rvsm);
    reportVelocityStateMessageDestroy(rvsm);
    
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(velocityStateSensorComponent, jMsg);
    }
    
    // MENSAJE ADDITIONAL GPS/INS INFO
    // Formacion del mensaje
    AditionalGPSINSInfo4Message agim = aditionalGPSINSInfo4MessageCreate();
    agim->presenceVector = 0x0007;
    agim->longitudinal_acc = msg.accX;
    agim->lateral_acc = msg.accY;
    agim->vertical_acc = msg.accZ;
    // Estado IMU y GPS
    // TODO!!!!!!!!!!!!!!!!!!!!!!!!
    
    // Direccion
    jAdd->component = JAUS_GLOBAL_POSE_SENSOR;;
    
    jausAddressCopy(agim->destination, jAdd);
    jMsg = aditionalGPSINSInfo4MessageToJausMessage(agim);
    aditionalGPSINSInfo4MessageDestroy(agim);
    
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(velocityStateSensorComponent, jMsg);
    }
    
    
    // Destruccion del mensaje y direccion
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION CAMARA IR
 * CORRESPONDENCIA JAUS: REPORT NIGHT-TIME CAMERA
 */
void RosNode_Communications::fcn_subs_irCameraInfo(CITIUS_Control_Communication::msg_irinfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion camara IR");
    
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromIRCameraInfo(this->subsystemController,this->nodeController,msg.currentDZoom,msg.currentPolarity);
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(this->visualSensorComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    }else{
        ROS_INFO("[Control] Communications - No se ha podido generar mensaje JAUS con informacion de camara IR");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION TELEMETRO
 * CORRESPONDENCIA JAUS: TELEMETER INFO
 */
void RosNode_Communications::fcn_subs_telemeterInfo(CITIUS_Control_Communication::msg_echoesFound msg) {
    ROS_INFO("[Control] Communications - Recibida informacion telemetro");
    
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromTelemeterInfo(this->subsystemController,this->nodeController, msg.echoesFound);
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(this->platformSensorComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    }else{
        ROS_INFO("[Control] Communications - No se ha podido generar mensaje JAUS con informacion de telemetro");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION CAMARA TV
 * CORRESPONDENCIA JAUS: REPORT DAY-TIME CAMERA INFO
 */
void RosNode_Communications::fcn_subs_tvCameraInfo(CITIUS_Control_Communication::msg_tvinfo msg) {
    ROS_INFO("[Control] Communications - Recibida informacion camara diurna (TV)");
    
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromTVCamera(this->subsystemController,this->nodeController, msg.currentZoom, msg.currentFocus, msg.autofocusMode);
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(this->visualSensorComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    }else{
        ROS_INFO("[Control] Communications - No se ha podido generar mensaje JAUS camara diurna (TV)");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION POSITIONER
 * CORRESPONDENCIA JAUS: REPORT POSITIONER
 */
void RosNode_Communications::fcn_subs_positionerInfo(CITIUS_Control_Communication::msg_panTiltPosition msg) {
    ROS_INFO("[Control] Communications - Recibida informacion posicionador");
    
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromPositioner(this->subsystemController,this->nodeController, msg.panPosition, msg.tiltPosition);
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(this->visualSensorComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    }else{
        ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de posicionador");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}

/* 
 * INFORMACION DE MODO DE OPERACION
 * CORRESPONDENCIA JAUS: REPORT MISSION STATUS
 */

void RosNode_Communications::informStatus() {
    
    // Obtencion del estado
    int status;
    ros::NodeHandle nh;
    nh.getParam("vehicleStatus",status);
    JausMessage jMsg = NULL;
    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_MISSION_SPOOLER;

    // Generacion de mensaje especifico UGV Info
    ReportMissionStatusMessage rmsm = reportMissionStatusMessageCreate();
    rmsm->missionId = status;
    jausAddressCopy(rmsm->destination, jAdd);
    
    // Generacion de mensaje JUAS global
    jMsg = reportMissionStatusMessageToJausMessage(rmsm);
    
    if (jMsg != NULL) {
        ojCmptSendMessage(this->missionSpoolerComponent, jMsg);
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

void RosNode_Communications::fcn_receive_run_mission(OjCmpt cmp, JausMessage msg) {
    RunMissionMessage rMission = runMissionMessageFromJausMessage(msg);
    CITIUS_Control_Communication::srv_vehicleStatus vehicleStatus;
    
    // Desde comunicaciones no se puede cambiar a otro modo que no sea conduccion
    // u observacion
    
    short tempStatus = rMission->missionId;
    
    switch(tempStatus){
        
        case 5: // Conduccion (Valor de ICD)
            vehicleStatus.request.status = OPERATION_MODE_CONDUCCION;
            this->clientStatus.call(vehicleStatus);
            this->informStatus();
            break;
            
        case 6: // Observacion (Valor de ICD)
            vehicleStatus.request.status = OPERATION_MODE_OBSERVACION;
            this->clientStatus.call(vehicleStatus);
            this->informStatus();
            break;
            
        default:
            break;
    }

    // Liberacion de memoria
    runMissionMessageDestroy(rMission);
}

// Componente Primitive Driver

void RosNode_Communications::fcn_receive_set_wrench_effort(OjCmpt cmp, JausMessage msg) {
    SetWrenchEffortMessage sWrenchEffort = setWrenchEffortMessageFromJausMessage(msg);
    CITIUS_Control_Communication::msg_command command;
    
    // Comprobacion de direccion
    if((sWrenchEffort->presenceVector & PRESENCE_VECTOR_STEER) == PRESENCE_VECTOR_STEER){
        command.id_device = STEERING;
        command.value = sWrenchEffort->propulsiveRotationalEffortZPercent;
        pubCommand.publish(command);
    }
    
    // Comprobacion de acelerador
    if((sWrenchEffort->presenceVector & PRESENCE_VECTOR_THROTTLE) == PRESENCE_VECTOR_THROTTLE){
        command.id_device = THROTTLE;
        command.value = sWrenchEffort->propulsiveLinearEffortXPercent;
        pubCommand.publish(command);
    }
    
    // Comprobacion de freno de servicio
    if((sWrenchEffort->presenceVector & PRESENCE_VECTOR_BRAKE) == PRESENCE_VECTOR_BRAKE){
        command.id_device = BRAKE;
        command.value = sWrenchEffort->resistiveLinearEffortXPercent;
        pubCommand.publish(command);
    }
    
    // Liberacion de memoria
    setWrenchEffortMessageDestroy(sWrenchEffort);
}

void RosNode_Communications::fcn_receive_set_discrete_devices(OjCmpt cmp, JausMessage msg) {
    SetDiscreteDevicesMessage sDiscreteDevice = setDiscreteDevicesMessageFromJausMessage(msg);
    CITIUS_Control_Communication::msg_command command;
    
    // Comprobacion de freno de estacionamiento
    if((sDiscreteDevice->presenceVector & PRESENCE_VECTOR_PARKING_BRAKE)==PRESENCE_VECTOR_PARKING_BRAKE){
        command.id_device = HANDBRAKE;
        command.value = sDiscreteDevice->parkingBrake;
        pubCommand.publish(command);
    }
    
    // Comprobacion de marcha
    if((sDiscreteDevice->presenceVector & PRESENCE_VECTOR_GEAR)==PRESENCE_VECTOR_GEAR){
        command.id_device = GEAR;
        command.value = sDiscreteDevice->gear;
        pubCommand.publish(command);
    }

    // Comprobacion arranque/parada de motor
    // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    
    // Liberacion de memoria
    setDiscreteDevicesMessageDestroy(sDiscreteDevice);

}

// Componente Visual Sensor

void RosNode_Communications::fcn_receive_set_camera_pose(OjCmpt cmp, JausMessage msg) {
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
        
        this->pubCtrlFrontCamera.publish(fcCommand);
        
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
        
        this->pubCtrlRearCamera.publish(rcCommand);
        
    }
    
    // Liberacion de memoria
    setCameraPoseMessageDestroy(sCameraPose);
}

void RosNode_Communications::fcn_receive_set_signaling_elements(OjCmpt cmp, JausMessage msg) {
    SetSignalingElements18Message sSignaling = setSignalingElements18MessageFromJausMessage(msg);
    CITIUS_Control_Communication::msg_command command;
    
    // Comprobacion de luces de posicion
    if((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSP) == PRESENCE_VECTOR_DIPSP){
        command.id_device = DIPSP;
        command.value = sSignaling->dipsp;
        this->pubCommand.publish(command);
    }
    
    // Comprobacion de luces cortas
    if((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSS) == PRESENCE_VECTOR_DIPSS){
        command.id_device = DIPSS;
        command.value = sSignaling->dipss;
        this->pubCommand.publish(command);
    }
    
    // Comprobacion de luces largas
    if((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSR) == PRESENCE_VECTOR_DIPSR){
        command.id_device = DIPSR;
        command.value = sSignaling->dipsr;
        this->pubCommand.publish(command);
    }
    
    // Comprobacion de intermitente derecha
    if((sSignaling->presenceVector & PRESENCE_VECTOR_BLINKER_RIGHT) == PRESENCE_VECTOR_BLINKER_RIGHT){
        command.id_device = BLINKER_RIGHT;
        command.value = sSignaling->blinker_right;
        this->pubCommand.publish(command);
    }
    
    // Comprobacion de intermitente izquierda
    if((sSignaling->presenceVector & PRESENCE_VECTOR_BLINKER_LEFT) == PRESENCE_VECTOR_BLINKER_LEFT){
        command.id_device = BLINKER_LEFT;
        command.value = sSignaling->blinker_left;
        this->pubCommand.publish(command);
    }
    
    // Comprobacion de claxon
    if((sSignaling->presenceVector & PRESENCE_VECTOR_KLAXON) == PRESENCE_VECTOR_KLAXON){
        command.id_device = KLAXON;
        command.value = sSignaling->klaxon;
        this->pubCommand.publish(command);
    }
    
    // Liberacion de memoria
    setSignalingElements18MessageDestroy(sSignaling);
}

void RosNode_Communications::fcn_receive_set_positioner(OjCmpt cmp, JausMessage msg) {

}

void RosNode_Communications::fcn_receive_set_day_time_camera(OjCmpt cmp, JausMessage msg) {

}

void RosNode_Communications::fcn_receive_set_night_time_camera(OjCmpt cmp, JausMessage msg) {

}

// Componente Velocity State Sensor

void RosNode_Communications::fcn_receive_set_travel_speed(OjCmpt cmp, JausMessage msg) {
    SetTravelSpeedMessage sTravelSpeed = setTravelSpeedMessageFromJausMessage(msg);
    CITIUS_Control_Communication::msg_command command;
    
    // No hay comprobacion de presence vector. Un solo parametro
    command.id_device = CRUISING_SPEED;
    command.value = sTravelSpeed->speedMps;
    this->pubCommand.publish(command);
    
    // Liberacion de memoria
    setTravelSpeedMessageDestroy(sTravelSpeed);
}

// Componente HeartBeat Information

void RosNode_Communications::fcn_receive_heartbeat_channel_state(OjCmpt cmp, JausMessage msg) {

}

void RosNode_Communications::fcn_receive_heartbeat_position_info(OjCmpt cmp, JausMessage msg) {

}
