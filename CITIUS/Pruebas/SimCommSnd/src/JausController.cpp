
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
    subsystemController = 3; // UGV
    nodeController = 1; // Control
}

/*******************************************************************************
 *******************************************************************************
 *                     INICIALIZACION DE ARTEFACTOS JAUS                       *
 *******************************************************************************
 ******************************************************************************/

void JausController::initJAUS() {
    
    // Inicializacion de JAUS
    configData = new FileLoader("nodeManager.conf");
    handler = new JausHandler();
    /*
    try {
        
        configData = new FileLoader("nodeManager.conf");
        handler = new JausHandler();
        nm = new NodeManager(this->configData, this->handler);
        
    }catch(...){
        
        cout << "No se ha podido inicializar JAUS" << endl;
        
    }
    */ // DECOMENTAR CUANDO TENGA NODEMANAGER PROPIO
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
                
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(missionSpoolerComponent, JAUS_MISSION_SPOOLER, JAUS_RUN_MISSION, 0xFF);
        
    }
    
    // Primitive Driver
    primitiveDriverComponent = ojCmptCreate((char *) "Primitive Driver", JAUS_PRIMITIVE_DRIVER, 1);
    if (primitiveDriverComponent == NULL) {
        cout << "No se ha podido crear el componente PRIMITIVE DRIVER" << endl;
        exit(0);
    }else{
    
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_SET_WRENCH_EFFORT, 0xFF);
        ojCmptAddServiceOutputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_SET_DISCRETE_DEVICES, 0xFF);
    
    }
    
    // Visual Sensor
    visualSensorComponent = ojCmptCreate((char *) "Visual Sensor", JAUS_VISUAL_SENSOR, 1);
    if (visualSensorComponent == NULL) {
        cout << "No se ha podido crear el componente VISUAL SENSOR" << endl;
        exit(0);
    }else{
        
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_CAMERA_POSE, 0xFF);
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_SIGNALING_ELEMENTS_18, 0xFF);
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_POSITIONER_19, 0xFF);
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_DAY_TIME_CAMERA_21, 0xFF);
        ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_NIGHT_TIME_CAMERA_23, 0xFF);
        
    }
    
    // Platform Sensor
    platformSensorComponent = ojCmptCreate((char *) "Platform Sensor", JAUS_PLATFORM_SENSOR, 1);
    if (platformSensorComponent == NULL) {
        cout << "No se ha podido crear el componente PLATFORM SENSOR" << endl;
        exit(0);
    }else{
        
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(platformSensorComponent, JAUS_PLATFORM_SENSOR, JAUS_SET_TELEMETER_26, 0xFF);
        
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
        
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_SET_TRAVEL_SPEED, 0xFF);
    
    }
    
    // Global Pose Sensor
    globalPoseSensorComponent = ojCmptCreate((char *)"Global Pose Sensor",JAUS_GLOBAL_POSE_SENSOR,1);
    if (globalPoseSensorComponent == NULL) {
        cout << "No se ha podido crear el componente GLOBAL POSE SENSOR"<< endl;
        exit(0);
    }else{
        // Mensajes que envia
        
    }
    
    // HeartBeat Information
    heartBeatInformationComponent = ojCmptCreate((char *)"HeartBeat Information", JAUS_HEARTBEAT_INFORMATION,1);
    if (heartBeatInformationComponent == NULL) {
        cout << "No se ha podido crear el componente HEARTBEAT INFORMATION" << endl;
        exit(0);
    }else{
        // Mensajes que envia
        ojCmptAddServiceOutputMessage(heartBeatInformationComponent, JAUS_HEARTBEAT_INFORMATION, JAUS_HEARTBEAT_CHANNEL_STATE_16, 0xFF);
        ojCmptAddServiceOutputMessage(heartBeatInformationComponent, JAUS_HEARTBEAT_INFORMATION, JAUS_HEARTBEAT_POSITION_INFO_17, 0xFF);
        
    }
    
    // Run de componentes
    ojCmptRun(missionSpoolerComponent);
    ojCmptRun(primitiveDriverComponent);
    ojCmptRun(visualSensorComponent);
    ojCmptRun(platformSensorComponent);
    ojCmptRun(globalWaypointDriverComponent);
    ojCmptRun(velocityStateSensorComponent);
    ojCmptRun(globalPoseSensorComponent);
    ojCmptRun(heartBeatInformationComponent);
}

/*******************************************************************************
 *******************************************************************************
 *                       FINALIZACION DE ARTEFACTOS JAUS                       *
 *******************************************************************************
 ******************************************************************************/

void JausController::endJAUS(){
    ojCmptDestroy(missionSpoolerComponent);
    ojCmptDestroy(primitiveDriverComponent);
    ojCmptDestroy(visualSensorComponent);
    ojCmptDestroy(platformSensorComponent);
    ojCmptDestroy(globalWaypointDriverComponent);
    ojCmptDestroy(velocityStateSensorComponent);
    ojCmptDestroy(globalPoseSensorComponent);
    ojCmptDestroy(heartBeatInformationComponent);
}

void JausController::sendMessage(int identifier){
    switch(identifier){
        case 1: // Run Mission
            sendRunMissionMessage();
            break;
        case 2: // Set Wrench Effort
            sendWrenchEffortMessage();
            break;
        case 3: // Set Discrete devices
            sendDiscreteDeviceMessage();
            break;
        case 4: // Set Travel Spped
            sendTravelSpeedMessage();
            break;
        case 5: // Set Camera Pose
            sendCameraPoseMessage();
            break;
        case 6: // Pause Mission
            sendPauseMissionMessage();
            break;
        case 7: // Resume Mission
            sendResumeMissionMessage();
            break;
        case 8: // Telemeter Info
            sendTelemeterInfoMessage();
            break;
        case 9: // Set Signaling Elements
            sendSignalingElementsMessage();
        case 10: // Set Positioner
            sendPositionerMessage();
            break;
        case 11: // Set Day-time camera
            sendTVCameraMessage();
            break;
        case 12: // Set Night-time camera
            sendIRCameraMessage();
            break;
    }
}

// Run Mission
void JausController::sendRunMissionMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_MISSION_SPOOLER;
    jAdd->instance = 1;
    
    RunMissionMessage spfMsg = runMissionMessageCreate();
    
    // Campo Mission ID
    int missionID;
    cout << "Selecciona el MissionID a enviar:" << endl;
    cin >> missionID;
    spfMsg->missionId = missionID;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = runMissionMessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->missionSpoolerComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    runMissionMessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje RUN MISSION" << endl;
}

// Set Wrench Effort
void JausController::sendWrenchEffortMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_PRIMITIVE_DRIVER;
    jAdd->instance = 1;
    
    SetWrenchEffortMessage spfMsg = setWrenchEffortMessageCreate();
    
    // Campo direccion
    int intAux;
    cout << "Selecciona un valor para la direccion (-100:100):" << endl;
    cin >> intAux;
    spfMsg->propulsiveRotationalEffortZPercent = intAux;
    
    // Campo acelerador
    cout << "Selecciona un valor para el acelerador (-100:100):" << endl;
    cin >> intAux;
    spfMsg->propulsiveLinearEffortXPercent = intAux;
    
    // Campo freno de servicio
    cout << "Selecciona un valor para el freno de servicio (0:100):" << endl;
    cin >> intAux;
    spfMsg->resistiveLinearEffortXPercent = intAux;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = setWrenchEffortMessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->primitiveDriverComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    setWrenchEffortMessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje SET WRENCH EFFORT" << endl;
}

// Set Discrete Devices
void JausController::sendDiscreteDeviceMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_PRIMITIVE_DRIVER;
    jAdd->instance = 1;
    
    SetDiscreteDevicesMessage spfMsg = setDiscreteDevicesMessageCreate();
    
    // Campo marcha
    int intAux;
    cout << "Selecciona un valor para la marcha {0,1,2}:" << endl;
    cin >> intAux;
    spfMsg->gear = intAux;
    
    // Freno de estacionamiento
    cout << "Selecciona un valor para el freno de servicio {0,1}:" << endl;
    cin >> intAux;
    if(intAux == 1) spfMsg->parkingBrake = JAUS_TRUE;
    else spfMsg->parkingBrake = JAUS_FALSE; 
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = setDiscreteDevicesMessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->primitiveDriverComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    setDiscreteDevicesMessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje SET DISCRETE DEVICE" << endl;
}

// Set Travel Speed
void JausController::sendTravelSpeedMessage(){
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_VELOCITY_STATE_SENSOR;
    jAdd->instance = 1;
    
    SetTravelSpeedMessage spfMsg = setTravelSpeedMessageCreate();
    
    // Campo velocidad de crucero
    float flAux;
    cout << "Selecciona un valor para la velocidad de crucero (0:10000):" << endl;
    cin >> flAux;
    spfMsg->speedMps = flAux;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = setTravelSpeedMessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->velocityStateSensorComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    setTravelSpeedMessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje SET TRAVEL SPEED" << endl;
}

// Set Camera Pose
void JausController::sendCameraPoseMessage(){
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 1;
    
    SetCameraPoseMessage spfMsg = setCameraPoseMessageCreate();
    
    // Pan
    int intAux;
    cout << "Selecciona un valor para PAN (-100:100):" << endl;
    cin >> intAux;
    spfMsg->zAngularPositionOrRatePercent = intAux;
    
    // Tilt
    cout << "Selecciona un valor para TILT (-100:100):" << endl;
    cin >> intAux;
    spfMsg->yAngularPositionOrRatePercent = intAux;
    
    // Zoom
    cout << "Selecciona un valor para ZOOM (-100:100):" << endl;
    cin >> intAux;
    spfMsg->zLinearPositionOrRatePercent = intAux;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = setCameraPoseMessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->visualSensorComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    setCameraPoseMessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje SET CAMERA POSE" << endl;
}

// Pause Mission
void JausController::sendPauseMissionMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_MISSION_SPOOLER;
    jAdd->instance = 1;
    
    PauseMissionMessage spfMsg = pauseMissionMessageCreate();
    
    // Campo Mission ID
    int missionID;
    cout << "Selecciona el MissionID a enviar:" << endl;
    cin >> missionID;
    spfMsg->missionId = missionID;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = pauseMissionMessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->missionSpoolerComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    pauseMissionMessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje PAUSE MISSION" << endl;
}

// Resume Mission
void JausController::sendResumeMissionMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_MISSION_SPOOLER;
    jAdd->instance = 1;
    
    ResumeMissionMessage spfMsg = resumeMissionMessageCreate();
    
    // Campo Mission ID
    int missionID;
    cout << "Selecciona el MissionID a enviar:" << endl;
    cin >> missionID;
    spfMsg->missionId = missionID;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = resumeMissionMessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->missionSpoolerComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    resumeMissionMessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje RESUME MISSION" << endl;
}

// Telemeter Info
void JausController::sendTelemeterInfoMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_PLATFORM_SENSOR;
    jAdd->instance = 1;
    
    SetTelemeter26Message spfMsg = setTelemeter26MessageCreate();
    
    // Campo shoot
    spfMsg->shoot = JAUS_TRUE;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = setTelemeter26MessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->platformSensorComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    setTelemeter26MessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje SET TELEMETER" << endl;
}  

// Set Signaling Elements
void JausController::sendSignalingElementsMessage(){
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 1;
    
    SetSignalingElements18Message spfMsg = setSignalingElements18MessageCreate();
    int intAux;
    // Intermitente derecha
    cout << "Selecciona un valor para el intermitente derecho {0,1}:" << endl;
    cin >> intAux;
    if(intAux == 1) spfMsg->blinker_right = JAUS_TRUE;
    else spfMsg->blinker_right = JAUS_FALSE; 

    // Intermitente izquierda
    cout << "Selecciona un valor para el intermitente izquierdo {0,1}:" << endl;
    cin >> intAux;
    if(intAux == 1) spfMsg->blinker_left = JAUS_TRUE;
    else spfMsg->blinker_left = JAUS_FALSE; 

    // Luces de posicion
    cout << "Selecciona un valor para luces de posicion {0,1}:" << endl;
    cin >> intAux;
    if(intAux == 1) spfMsg->dipsp = JAUS_TRUE;
    else spfMsg->dipsp = JAUS_FALSE; 

    // Luces cortas
    cout << "Selecciona un valor para luces cortas {0,1}:" << endl;
    cin >> intAux;
    if(intAux == 1) spfMsg->dipss = JAUS_TRUE;
    else spfMsg->dipss = JAUS_FALSE; 

    // Luces largas
    cout << "Selecciona un valor para luces largas {0,1}:" << endl;
    cin >> intAux;
    if(intAux == 1) spfMsg->dipsr = JAUS_TRUE;
    else spfMsg->dipsr = JAUS_FALSE; 

    // Bocina
    cout << "Selecciona un valor para bocina {0,1}:" << endl;
    cin >> intAux;
    if(intAux == 1) spfMsg->klaxon = JAUS_TRUE;
    else spfMsg->klaxon = JAUS_FALSE; 
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = setSignalingElements18MessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->visualSensorComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    setSignalingElements18MessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje SET DISCRETE DEVICE" << endl;
}

// Set Positioner
void JausController::sendPositionerMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 1;
    
    SetPositioner19Message spfMsg = setPositioner19MessageCreate();
    
    // Pan
    int intAux;
    cout << "Selecciona un valor para PAN (0:6399):" << endl;
    cin >> intAux;
    spfMsg->pan = intAux;
    
    // Tilt
    cout << "Selecciona un valor para TILT (-1600:1600):" << endl;
    cin >> intAux;
    spfMsg->tilt = intAux;
    
    // Velocidad de giro
    cout << "Selecciona un valor para VELOCIDAD DE GIRO (-100:100):" << endl;
    cin >> intAux;
    spfMsg->spin_velocity = intAux;
    
    // Velocidad de elevacion
    cout << "Selecciona un valor para VELOCIDAD DE ELEVACION (-100:100):" << endl;
    cin >> intAux;
    spfMsg->elevation_velocity = intAux;

    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = setPositioner19MessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->visualSensorComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    setPositioner19MessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje SET POSITIONER" << endl;
}

// Set Day-time camera
void JausController::sendTVCameraMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 1;
    
    SetDayTimeCamera21Message spfMsg = setDayTimeCamera21MessageCreate();
    
    // Zoom Directo
    int intAux;
    cout << "Selecciona un valor para ZOOM DIRECTO (0:100):" << endl;
    cin >> intAux;
    spfMsg->direct_zoom = intAux;
    
    // Zoom continuo
    cout << "Selecciona un valor para ZOOM CONTINUO {0,1,2}:" << endl;
    cin >> intAux;
    spfMsg->continuous_zoom = intAux;
    
    // Foco
    cout << "Selecciona un valor para FOCO (0:100):" << endl;
    cin >> intAux;
    spfMsg->focus = intAux;
    
    // Autofoco
    cout << "Selecciona un valor para AUTOFOCO {0,1}:" << endl;
    cin >> intAux;
    if(intAux == 1) spfMsg->autofocus = JAUS_TRUE;
    else spfMsg->autofocus = JAUS_FALSE;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = setDayTimeCamera21MessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->visualSensorComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    setDayTimeCamera21MessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje SET DAY-TIME CAMERA" << endl;
}

// Set Night-time camera
void JausController::sendIRCameraMessage(){
    
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subsystemController;
    jAdd->node = nodeController;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 1;
    
    SetNightTimeCamera23Message spfMsg = setNightTimeCamera23MessageCreate();

    // Zoom
    int intAux;
    cout << "Selecciona un valor para ZOOM {0,1,2}:" << endl;
    cin >> intAux;
    spfMsg->zoom = intAux;
    
    // Polatidad
    cout << "Selecciona un valor para POLARIDAD {0,1}:" << endl;
    cin >> intAux;
    if(intAux == 1) spfMsg->polarity = JAUS_TRUE;
    else spfMsg->polarity = JAUS_FALSE;
    
    jausAddressCopy(spfMsg->destination,jAdd);
    jMsg = setNightTimeCamera23MessageToJausMessage(spfMsg);
    if (jMsg != NULL) {
        ojCmptSendMessage(instance->visualSensorComponent, jMsg);
    } else {
        cout << "No se ha podido generar mensaje JAUS" << endl;
    }
    // Liberacion de memoria
    setNightTimeCamera23MessageDestroy(spfMsg);
    jausAddressDestroy(jAdd);
    jausMessageDestroy(jMsg);
    cout << "Enviado mensaje SET NIGHT-TIME CAMERA" << endl;
}