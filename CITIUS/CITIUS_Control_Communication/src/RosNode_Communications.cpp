
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
    
    //Primitive Driver
    primitiveDriverComponent = ojCmptCreate((char *) "Primitive Driver", JAUS_PRIMITIVE_DRIVER, 1);
    if (primitiveDriverComponent == NULL) {
        ROS_INFO("[Contorl] Communication - No se ha podido crear el componente PRIMITIVE DRIVER");
        exit(0);
    }
    
    // Mission Spooler
    missionSpoolerComponent = ojCmptCreate((char *) "Mission Spooler", JAUS_MISSION_SPOOLER, 1);
    if (missionSpoolerComponent == NULL) {
        ROS_INFO("[Contorl] Communication - No se ha podido crear el componente MISSION SPOOLER");
        exit(0);
    }
    
    // Visual Sensor
    visualSensorComponent = ojCmptCreate((char *) "Visual Sensor", JAUS_VISUAL_SENSOR, 1);
    if (visualSensorComponent == NULL) {
        ROS_INFO("[Contorl] Communication - No se ha podido crear el componente VISUAL SENSOR");
        exit(0);
    }
    
    // Platform Sensor
    platformSensorComponent = ojCmptCreate((char *) "Platform Sensor", JAUS_PLATFORM_SENSOR, 1);
    if (platformSensorComponent == NULL) {
        ROS_INFO("[Contorl] Communication - No se ha podido crear el componente PLATFORM SENSOR");
        exit(0);
    }
    
    // Global Waypoint Driver
    globalWaypointDriverComponent = ojCmptCreate((char *)"Global Waypoint Driver", JAUS_GLOBAL_WAYPOINT_DRIVER,1);
    if (globalWaypointDriverComponent == NULL) {
        ROS_INFO("[Contorl] Communication - No se ha podido crear el componente GLOBAL WAYPOINT DRIVER");
        exit(0);
    }
    
    // Velocity State Sensor
    velocityStateSensorComponent = ojCmptCreate((char *)"Velocity State Sensor",JAUS_VELOCITY_STATE_SENSOR,1);
    if (velocityStateSensorComponent == NULL) {
        ROS_INFO("[Contorl] Communication - No se ha podido crear el componente VELOCITY STATE SENSOR");
        exit(0);
    }
    
    // Global Pose Sensor
    globalPoseSensorComponent = ojCmptCreate((char *)"Global Pose Sensor",JAUS_GLOBAL_POSE_SENSOR,1);
    if (globalPoseSensorComponent == NULL) {
        ROS_INFO("[Contorl] Communication - No se ha podido crear el componente GLOBAL POSE SENSOR");
        exit(0);
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
    ROS_INFO("[Control] Communications - Recibida informacion de posicionamiento");
    
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromPosOriInfo(this->subsystemController,this->nodeController,msg.latitude,msg.longitude,msg.altitude,msg.roll,msg.pitch,msg.yaw);
    if(jMsg != NULL){
        // Envio via JAUS    
        ojCmptSendMessage(this->globalPoseSensorComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    }else{
        ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion electrica de vehiculo");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}

// Informa a Controller del estado

void RosNode_Communications::informStatus() {
    ROS_INFO("[Control] Communications - Comunicando modo de operacion al controlador");
    
    // Obtencion del estado
    int status;
    ros::NodeHandle nh;
    nh.getParam("vehicleStatus",status);
    // Conversor ROS -> JAUS
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    // Creacion del mensaje a enviar
    JausMessage jMsg = translator->getJausMsgFromStatus(this->subsystemController, this->nodeController, status);
    if (jMsg != NULL) {
        // Envio via JAUS    
        ojCmptSendMessage(this->missionSpoolerComponent, jMsg);
        ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    } else {
        ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion electrica de vehiculo");
    }
    // Destruccion del mensaje
    jausMessageDestroy(jMsg);
}
