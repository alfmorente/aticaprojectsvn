
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
        // TODO!!!!!!!!!!!!!!!
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
 *                               CALLBACKS                                     *
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
    JausMessage jMsg = translator->getJausMsgFromCameraInfo(this->subsystemController,this->nodeController,FRONT_CAMERA_ID,msg.pan,msg.tilt);
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
    JausMessage jMsg = translator->getJausMsgFromCameraInfo(this->subsystemController,this->nodeController,REAR_CAMERA_ID,msg.pan,msg.tilt);
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

}

// Componente Primitive Driver

void RosNode_Communications::fcn_receive_set_wrench_effort(OjCmpt cmp, JausMessage msg) {

}

void RosNode_Communications::fcn_receive_set_discrete_devices(OjCmpt cmp, JausMessage msg) {

}

// Componente Visual Sensor

void RosNode_Communications::fcn_receive_set_camera_pose(OjCmpt cmp, JausMessage msg) {

}

void RosNode_Communications::fcn_receive_set_signaling_elements(OjCmpt cmp, JausMessage msg) {

}

void RosNode_Communications::fcn_receive_set_positioner(OjCmpt cmp, JausMessage msg) {

}

void RosNode_Communications::fcn_receive_set_day_time_camera(OjCmpt cmp, JausMessage msg) {

}

void RosNode_Communications::fcn_receive_set_night_time_camera(OjCmpt cmp, JausMessage msg) {

}

// Componente Velocity State Sensor

void RosNode_Communications::fcn_receive_set_travel_speed(OjCmpt cmp, JausMessage msg) {

}

// Componente HeartBeat Information

void RosNode_Communications::fcn_receive_heartbeat_channel_state(OjCmpt cmp, JausMessage msg) {

}

void RosNode_Communications::fcn_receive_heartbeat_position_info(OjCmpt cmp, JausMessage msg) {

}
