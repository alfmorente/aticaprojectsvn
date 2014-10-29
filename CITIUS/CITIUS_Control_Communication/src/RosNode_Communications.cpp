
/** 
 * @file  RosNode_Communications.cpp
 * @brief Implementación de la clase "RosNode_Communications"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode_Communications.h"

/// Declaracion para patron Singleton
bool RosNode_Communications::instanceCreated = false;
RosNode_Communications *RosNode_Communications::instance = NULL;

/** 
 * Método público utilizado para el uso del patron Singleton. Aseguramiento de 
 * una única instancia de la clase.
 * @return Instancia de la clase "RosNode_Communications" si no existiera
 */
RosNode_Communications *RosNode_Communications::getInstance() {
  if (!instanceCreated) {
    instance = new RosNode_Communications();
    instanceCreated = true;
  }
  return instance;
}

/** 
 * Constructor de la clase
 */
RosNode_Communications::RosNode_Communications() {
  subsystemController = JAUS_SUBSYSTEM_MYC; // UGV
    //subsystemController = JAUS_SUBSYSTEM_UGV;
    nodeController = JAUS_NODE_CONTROL; // Control
    nodeStatus = NODESTATUS_INIT;
}

/**
 * Destructor de la clase
 */
RosNode_Communications::~RosNode_Communications(){
    
}

/** 
 * Método público que inicia los artefactos ROS que la clase contiene en sus 
 * atributos 
 */
void RosNode_Communications::initROS() {
  ros::NodeHandle nh;
  // Inicializacion de publicadores
  pubCommand = nh.advertise<CITIUS_Control_Communication::msg_command>("command", 1000);
  pubCtrlFrontCamera = nh.advertise<CITIUS_Control_Communication::msg_ctrlFrontCamera>("ctrlFrontCamera", 1000);
  pubCtrlRearCamera = nh.advertise<CITIUS_Control_Communication::msg_ctrlRearCamera>("ctrlRearCamera", 1000);
  // Inicializacion de suscriptores
  //          Subsistema de control
  subsElectricInfo = nh.subscribe("electricInfo", 1000, &RosNode_Communications::fnc_subs_electricInfo, this);
  subsVehicleInfo = nh.subscribe("vehicleInfo", 1000, &RosNode_Communications::fnc_subs_vehicleInfo, this);
  subsFrontCameraInfo = nh.subscribe("frontCameraInfo", 1000, &RosNode_Communications::fnc_subs_frontCameraInfo, this);
  subsRearCameraInfo = nh.subscribe("rearCameraInfo", 1000, &RosNode_Communications::fnc_subs_rearCameraInfo, this);
  subsPosOriInfo = nh.subscribe("posOriInfo", 1000, &RosNode_Communications::fnc_subs_posOriInfo, this);
  //          Subsistema de payload de observacion
  subsIRCameraInfo = nh.subscribe("IRInformation", 1000, &RosNode_Communications::fcn_subs_irCameraInfo, this);
  subsTelemeterInfo = nh.subscribe("LRFEchoesFound", 1000, &RosNode_Communications::fcn_subs_positionerInfo, this);
  subsTVCameraInfo = nh.subscribe("TVInformation", 1000, &RosNode_Communications::fcn_subs_tvCameraInfo, this);
  subsPositionerInfo = nh.subscribe("PanTiltPosition", 1000, &RosNode_Communications::fcn_subs_telemeterInfo, this);

  // Inicializacion de servicios
  //          Subsistema de control
  clientStatus = nh.serviceClient<CITIUS_Control_Communication::srv_vehicleStatus>("vehicleStatus");
  //          Subsistema de payload de observacion
  clientIRCameraPolarity = nh.serviceClient<CITIUS_Control_Communication::srv_polarity>("IRPolarity");
  clientIRCameraZoom = nh.serviceClient<CITIUS_Control_Communication::srv_dzoom>("IRDZoom");
  clientTVCameraDirectZoom = nh.serviceClient<CITIUS_Control_Communication::srv_zoomDirect>("TVZoomDirect");
  clientTVCameraContZoom = nh.serviceClient<CITIUS_Control_Communication::srv_zoomCommand>("TVZoomCommand");
  clientTVCameraFocus = nh.serviceClient<CITIUS_Control_Communication::srv_focusDirect>("TVFocusDirect");
  clientTVCameraAutofocus = nh.serviceClient<CITIUS_Control_Communication::srv_autofocusMode>("TVAutofocusMode");
  clientPosPanAbs = nh.serviceClient<CITIUS_Control_Communication::srv_panAbsolutePosition>("setPanPosition");
  clientPosPanRate = nh.serviceClient<CITIUS_Control_Communication::srv_panRate>("setPanRate");
  clientPosTiltAbs = nh.serviceClient<CITIUS_Control_Communication::srv_tiltAbsolutePosition>("setTiltPosition");
  clientPosTiltRate = nh.serviceClient<CITIUS_Control_Communication::srv_tiltRate>("setTiltRate");
  clientShootTel = nh.serviceClient<CITIUS_Control_Communication::srv_shoot>("LRFShoot");
  // Maquina de estados del nodo
  servNodeStatus = nh.advertiseService("cmNodeStatus", &RosNode_Communications::fcv_serv_nodeStatus, this);
}

/** 
 * Método público que inicia los artefactos JAUS que la clase contiene en sus 
 * atributos y puesta en marcha de los mismos
 */
void RosNode_Communications::initJAUS() {

  // Inicializacion de JAUS
  configData = new FileLoader("/home/ugv/catkin_ws/src/CITIUS_Control_Communication/bin/nodeManager.conf");
  handler = new JausHandler();

  try {
    configData = new FileLoader("/home/ugv/catkin_ws/src/CITIUS_Control_Communication/bin/nodeManager.conf");
    handler = new JausHandler();
    nm = new NodeManager(this->configData, this->handler);
  } catch (...) {
    //ROS_INFO("[Control] Communications - No se ha podido inicializar JAUS");
  }

  //Creacion de componentes
  // Mission Spooler
  missionSpoolerComponent = ojCmptCreate((char *) "Mission Spooler", JAUS_MISSION_SPOOLER, 1);
  if (missionSpoolerComponent == NULL) {
    //ROS_INFO("[Control] Communication - No se ha podido crear el componente MISSION SPOOLER");
    exit(0);
  } else {
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
    //ROS_INFO("[Control] Communication - No se ha podido crear el componente PRIMITIVE DRIVER");
    exit(0);
  } else {
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
    //ROS_INFO("[Control] Communication - No se ha podido crear el componente VISUAL SENSOR");
    exit(0);
  } else {
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
    //ROS_INFO("[Control] Communication - No se ha podido crear el componente PLATFORM SENSOR");
    exit(0);
  } else {
    // Mensajes que envia
    ojCmptAddServiceOutputMessage(platformSensorComponent, JAUS_PLATFORM_SENSOR, JAUS_REPORT_TELEMETER_27, 0xFF);
    // Mensajes que recibe
    ojCmptAddServiceInputMessage(platformSensorComponent, JAUS_PLATFORM_SENSOR, JAUS_SET_TELEMETER_26, 0xFF);
    // Funciones de recepcion de mensajes (Callbacks)
    ojCmptSetMessageCallback(platformSensorComponent, JAUS_SET_TELEMETER_26, fcn_receive_set_telemeter);

  }

  // Global Waypoint Driver
  globalWaypointDriverComponent = ojCmptCreate((char *) "Global Waypoint Driver", JAUS_GLOBAL_WAYPOINT_DRIVER, 1);
  if (globalWaypointDriverComponent == NULL) {
    //ROS_INFO("[Control] Communication - No se ha podido crear el componente GLOBAL WAYPOINT DRIVER");
    exit(0);
  } else {
    // REVISAR COMPONENTE INNECESARIO PARA UGV
    // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  }

  // Velocity State Sensor
  velocityStateSensorComponent = ojCmptCreate((char *) "Velocity State Sensor", JAUS_VELOCITY_STATE_SENSOR, 1);
  if (velocityStateSensorComponent == NULL) {
    //ROS_INFO("[Control] Communication - No se ha podido crear el componente VELOCITY STATE SENSOR");
    exit(0);
  } else {
    // Mensajes que envia
    ojCmptAddServiceOutputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_REPORT_TRAVEL_SPEED, 0xFF);
    ojCmptAddServiceOutputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_REPORT_VELOCITY_STATE, 0xFF);
    // Mensajes que recibe
    ojCmptAddServiceInputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_SET_TRAVEL_SPEED, 0xFF);
    // Funciones de recepcion de mensajes (Callbacks)
    ojCmptSetMessageCallback(velocityStateSensorComponent, JAUS_SET_TRAVEL_SPEED, fcn_receive_set_travel_speed);
  }

  // Global Pose Sensor
  globalPoseSensorComponent = ojCmptCreate((char *) "Global Pose Sensor", JAUS_GLOBAL_POSE_SENSOR, 1);
  if (globalPoseSensorComponent == NULL) {
    //ROS_INFO("[Control] Communication - No se ha podido crear el componente GLOBAL POSE SENSOR");
    exit(0);
  } else {
    // Mensajes que envia
    ojCmptAddServiceOutputMessage(globalPoseSensorComponent, JAUS_GLOBAL_POSE_SENSOR, JAUS_REPORT_GLOBAL_POSE, 0xFF);
    ojCmptAddServiceOutputMessage(globalPoseSensorComponent, JAUS_GLOBAL_POSE_SENSOR, JAUS_ADITIONAL_GPSINS_INFO_4, 0xFF);
  }

  // HeartBeat Information
  heartBeatInformationComponent = ojCmptCreate((char *) "HeartBeat Information", JAUS_HEARTBEAT_INFORMATION, 1);
  if (heartBeatInformationComponent == NULL) {
    //ROS_INFO("[Control] Communication - No se ha podido crear el componente HEARTBEAT INFORMATION");
    exit(0);
  } else {
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

/** 
 * Método público que hace de destructor de componentes JAUS de la clase para
 * su finalización
 */
void RosNode_Communications::finishJAUS() {
  ojCmptDestroy(missionSpoolerComponent);
  ojCmptDestroy(primitiveDriverComponent);
  ojCmptDestroy(visualSensorComponent);
  ojCmptDestroy(platformSensorComponent);
  ojCmptDestroy(globalWaypointDriverComponent);
  ojCmptDestroy(velocityStateSensorComponent);
  ojCmptDestroy(globalPoseSensorComponent);
  ojCmptDestroy(heartBeatInformationComponent);
}

/** 
 * Método público que obtiene el modo de operación activo de la máquina de 
 * estados, lo traduce a JAUS (Report Mission Status) y lo envía al controlador
 */
void RosNode_Communications::informStatus() {

  // Obtencion del estado
  int status;
  ros::NodeHandle nh;
  nh.getParam("vehicleStatus", status);
  JausMessage jMsg = NULL;

  // Creacion de la direccion destinataria
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = instance->subsystemController;
  jAdd->node = instance->nodeController;
  jAdd->component = JAUS_MISSION_SPOOLER;
  jAdd->instance = 1;

  // Generacion de mensaje especifico UGV Info
  ReportMissionStatusMessage rmsm = reportMissionStatusMessageCreate();
  if (status == OPERATION_MODE_LOCAL) {
    rmsm->missionId = JAUS_OPERATION_MODE_LOCAL;
  } else if (status == OPERATION_MODE_CONDUCCION) {
    rmsm->missionId = JAUS_OPERATION_MODE_CONDUCCION;
  } else if (status == OPERATION_MODE_OBSERVACION) {
    rmsm->missionId = JAUS_OPERATION_MODE_OBSERVACION;
  } else {
    rmsm->missionId = JAUS_OPERATION_MODE_INICIANDO;
  }
  rmsm->status = 0;

  jausAddressCopy(rmsm->destination, jAdd);

  // Generacion de mensaje JUAS global
  jMsg = reportMissionStatusMessageToJausMessage(rmsm);

  if (jMsg != NULL) {
    ojCmptSendMessage(instance->missionSpoolerComponent, jMsg);
  } else {
    ////ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion electrica de vehiculo");
  }
  // Liberacion de memoria
  reportMissionStatusMessageDestroy(rmsm);
  jausAddressDestroy(jAdd);
  jausMessageDestroy(jMsg);
}


/*******************************************************************************
 *******************************************************************************
 *                               CALLBACKS ROS                                 *
 *******************************************************************************
 ******************************************************************************/

/** 
 * Método privado consumidor de topic asociado a información de la cámara de 
 * apoyo a la conducción delantera. Traduce la información a JAUS y la envia al 
 * controlador
 * @param[in] msg Mensaje ROS con información de la cámara delantera
 */
void RosNode_Communications::fnc_subs_frontCameraInfo(CITIUS_Control_Communication::msg_frontCameraInfo msg) {
  //ROS_INFO("[Control] Communications - Recibida informacion de camara delantera");

  // Conversor ROS -> JAUS
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  // Creacion del mensaje a enviar
  JausMessage jMsg = translator->getJausMsgFromCameraInfo(this->subsystemController, this->nodeController, FRONT_CAMERA_ID, msg.pan, msg.tilt, msg.zoom);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de camara delantera");
  }
  // Destruccion del mensaje
  jausMessageDestroy(jMsg);
}


/**
 * Método privado encargado del tratamiento de servicios para la modificación de
 * la máquina de estados del nodo
 * @param[in] rq Parámetros de requerimiento
 * @param[in] rsp Parámetros de respuesta
 * @return Booleano que indica si se ha realizado el correcto tratamiento de
 * la petición  de servicio
 */
bool RosNode_Communications::fcv_serv_nodeStatus(CITIUS_Control_Communication::srv_nodeStatus::Request &rq, CITIUS_Control_Communication::srv_nodeStatus::Response &rsp) {
  if (rq.status == NODESTATUS_OK) {
    nodeStatus = NODESTATUS_OK;
    rsp.confirmation = true;
  } else if (rq.status == NODESTATUS_OFF) {
    nodeStatus = NODESTATUS_OFF;
    rsp.confirmation = true;
  } else {
    rsp.confirmation = false;
  }
  return true;
}

/** 
 * Método privado consumidor de topic asociado a información de la cámara de 
 * apoyo a la conducción trasera. Traduce la información a JAUS y la envia al 
 * controlador
 * @param[in] msg Mensaje ROS con información de la cámara trasera
 */
void RosNode_Communications::fnc_subs_rearCameraInfo(CITIUS_Control_Communication::msg_rearCameraInfo msg) {
  //ROS_INFO("[Control] Communications - Recibida informacion de camara trasera");

  // Conversor ROS -> JAUS
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  // Creacion del mensaje a enviar
  JausMessage jMsg = translator->getJausMsgFromCameraInfo(this->subsystemController, this->nodeController, REAR_CAMERA_ID, msg.pan, msg.tilt, msg.zoom);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de camara trasera");
  }
  // Destruccion del mensaje
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información del modulo de 
 * conducción del vehículo. Traduce la información a JAUS (Report Discrete 
 * Device/Report Wrench Effort/Report Signaling Elements/Report Travel Speed) y 
 * la envía al controlador
 * @param[in] msg Mensaje ROS con informacion del modulo de conduccion
 */
void RosNode_Communications::fnc_subs_vehicleInfo(CITIUS_Control_Communication::msg_vehicleInfo msg) {
  //ROS_INFO("[Control] Communications - Recibida informacion de vehiculo");

  // Conversor ROS -> JAUS
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();

  // Creacion del mensaje a enviar (Report Wrench Effort)
  JausMessage jMsg = translator->getJausMsgFromWrenchEffortInfo(this->subsystemController, this->nodeController, msg.steering, msg.thottle, msg.brake);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de vehiculo");
  }

  // Creacion del mensaje a enviar (Report Discrete Device)
  jMsg = translator->getJausMsgFromDiscreteDeviceInfo(this->subsystemController, this->nodeController, msg.parkingBrake, msg.gear);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de vehiculo");
  }

  // Creacion del mensaje a enviar (Report Travel Speed)
  jMsg = translator->getJausMsgFromTravelSpeedInfo(this->subsystemController, this->nodeController, msg.speed);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->velocityStateSensorComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de vehiculo");
  }

  // Creacion del mensaje a enviar (UGV Info)
  jMsg = translator->getJausMsgFromUGVInfo(this->subsystemController, this->nodeController, msg.motorRPM, msg.motorTemperature);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de vehiculo");
  }

  if (msg.lights) {
    // Creacion del mensaje a enviar (Report Signaling Elements)
    jMsg = translator->getJausMsgFromSignalingInfo(this->subsystemController, this->nodeController, msg.blinkerLeft, msg.blinkerRight, msg.dipsp, msg.dipss, msg.dipsr, msg.klaxon);
    if (jMsg != NULL) {
      // Envio via JAUS    
      ojCmptSendMessage(this->visualSensorComponent, jMsg);
      //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
    } else {
      //ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion de vehiculo");
    }
  }

  // Destruccion del mensaje
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información del módulo de 
 * gestión eléctrica del vehículo. Traduce la información a JAUS (UGV Info) y la 
 * envía al controlador
 * @param[in] msg Mensaje ROS con información del modulo de conducción
 */
void RosNode_Communications::fnc_subs_electricInfo(CITIUS_Control_Communication::msg_electricInfo msg) {
  //ROS_INFO("[Control] Communications - Recibida informacion electrica");

  // Conversor ROS -> JAUS
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  // Creacion del mensaje a enviar
  JausMessage jMsg = translator->getJausMsgFromElectricInfo(this->subsystemController, this->nodeController, msg.battery_level, msg.battery_voltage, msg.battery_current, msg.battery_temperature, msg.supply_alarms);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
  } else {
    //ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion electrica de vehiculo");
  }
  // Destruccion del mensaje
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información recopilada de los 
 * sensores de Posición/Orientación. Traduce la información a JAUS (Report 
 * Global Pose/Report Velocity State/Additional GPS/INS Info) y la envía al 
 * controlador
 * @param[in] msg Mensaje ROS con información de posición/orientación
 */
void RosNode_Communications::fnc_subs_posOriInfo(CITIUS_Control_Communication::msg_posOriInfo msg) {

  //ROS_INFO("[Control] Communications - Recibido mensaje de Position Orientation");
  JausMessage jMsg = NULL;

  // Creacion de la direccion destinataria
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subsystemController;
  jAdd->node = nodeController;
  jAdd->component = JAUS_GLOBAL_POSE_SENSOR;
  jAdd->instance = 2;

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

  if (jMsg != NULL) {
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
  jAdd->instance = 2;

  jausAddressCopy(rvsm->destination, jAdd);
  jMsg = reportVelocityStateMessageToJausMessage(rvsm);
  reportVelocityStateMessageDestroy(rvsm);

  if (jMsg != NULL) {
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
  jAdd->component = JAUS_GLOBAL_POSE_SENSOR;
  jAdd->instance = 2;

  jausAddressCopy(agim->destination, jAdd);
  jMsg = aditionalGPSINSInfo4MessageToJausMessage(agim);
  aditionalGPSINSInfo4MessageDestroy(agim);

  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(velocityStateSensorComponent, jMsg);
  }

  // Destruccion del mensaje y direccion
  jausAddressDestroy(jAdd);
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información recopilada de la 
 * cámara IR. Traduce la información a JAUS (Report Night-time Camera) y la 
 * envía al controlador
 * @param[in] msg Mensaje ROS con información de camara IR
 */
void RosNode_Communications::fcn_subs_irCameraInfo(CITIUS_Control_Communication::msg_irinfo msg) {
  //ROS_INFO("[Control] Communications - Recibida informacion camara IR");

  // Conversor ROS -> JAUS
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  // Creacion del mensaje a enviar
  JausMessage jMsg = translator->getJausMsgFromIRCameraInfo(this->subsystemController, this->nodeController, msg.currentDZoom, msg.currentPolarity);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha podido generar mensaje JAUS con informacion de camara IR");
  }
  // Destruccion del mensaje
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información recopilada del 
 * telémetro. Traduce la información a JAUS (Report Telemeter) y la envía al 
 * controlador
 * @param[in] msg Mensaje ROS con información de telemetro
 */
void RosNode_Communications::fcn_subs_telemeterInfo(CITIUS_Control_Communication::msg_echoesFound msg) {
  //ROS_INFO("[Control] Communications - Recibida informacion telemetro");

  // Conversor ROS -> JAUS
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  // Creacion del mensaje a enviar
  JausMessage jMsg = translator->getJausMsgFromTelemeterInfo(this->subsystemController, this->nodeController, msg.echoesFound.c_array());
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->platformSensorComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha podido generar mensaje JAUS con informacion de telemetro");
  }
  // Destruccion del mensaje
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información recopilada de la 
 * cámara TV. Traduce la información a JAUS (Report Day-time Camera) y la envía 
 * al controlador
 * @param[in] msg Mensaje ROS con información de camara TV
 */
void RosNode_Communications::fcn_subs_tvCameraInfo(CITIUS_Control_Communication::msg_tvinfo msg) {
  //ROS_INFO("[Control] Communications - Recibida informacion camara diurna (TV)");

  // Conversor ROS -> JAUS
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  // Creacion del mensaje a enviar
  JausMessage jMsg = translator->getJausMsgFromTVCamera(this->subsystemController, this->nodeController, msg.currentZoom, msg.currentFocus, msg.autofocusMode);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha podido generar mensaje JAUS camara diurna (TV)");
  }
  // Destruccion del mensaje
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información recopilada del 
 * posicionador. Traduce la información a JAUS (Report Positioner) y la envía al 
 * controlador
 * @param[in] msg Mensaje ROS con información del posicionador
 */
void RosNode_Communications::fcn_subs_positionerInfo(CITIUS_Control_Communication::msg_panTiltPosition msg) {
  //ROS_INFO("[Control] Communications - Recibida informacion posicionador");

  // Conversor ROS -> JAUS
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  // Creacion del mensaje a enviar
  JausMessage jMsg = translator->getJausMsgFromPositioner(this->subsystemController, this->nodeController, msg.panPosition, msg.tiltPosition);
  if (jMsg != NULL) {
    // Envio via JAUS    
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
    //ROS_INFO("[Control] Communications - Enviado mensaje via JAUS");
  } else {
    //ROS_INFO("[Control] Communications - No se ha podido generar mensaje JAUS con informacion de posicionador");
  }
  // Destruccion del mensaje
  jausMessageDestroy(jMsg);
}

/*******************************************************************************
 *******************************************************************************
 *                              CALLBACKS JAUS                                 *
 *******************************************************************************
 ******************************************************************************/

// Componente Mission Spooler

/** 
 * Método privado que recibe mensajes tipo JAUS "Run Mission". Solicita cambio 
 * de modo de operacióna la maquina de estados
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_run_mission(OjCmpt cmp, JausMessage msg) {
  RunMissionMessage rMission = runMissionMessageFromJausMessage(msg);
  CITIUS_Control_Communication::srv_vehicleStatus vehicleStatus;

  // Desde comunicaciones no se puede cambiar a otro modo que no sea conduccion
  // u observacion

  short tempStatus = rMission->missionId;

  switch (tempStatus) {

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

/** 
 * Método privado que recibe mensajes JAUS de tipo "Set Wrench Effort". Traduce 
 * la información a mensaje ROS (msg_command) y la publica en el topic 
 * correspondiente para la ejecución de comandos de control del vehiculo 
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_set_wrench_effort(OjCmpt cmp, JausMessage msg) {
  SetWrenchEffortMessage sWrenchEffort = setWrenchEffortMessageFromJausMessage(msg);
  CITIUS_Control_Communication::msg_command command;

  // Comprobacion de direccion
  if ((sWrenchEffort->presenceVector & PRESENCE_VECTOR_STEER) == PRESENCE_VECTOR_STEER) {
    command.id_device = STEERING;
    command.value = sWrenchEffort->propulsiveRotationalEffortZPercent;
    //pubCommand.publish(command);
    instance->pubCommand.publish(command);
  }

  // Comprobacion de acelerador
  if ((sWrenchEffort->presenceVector & PRESENCE_VECTOR_THROTTLE) == PRESENCE_VECTOR_THROTTLE) {
    command.id_device = THROTTLE;
    command.value = sWrenchEffort->propulsiveLinearEffortXPercent;
    instance->pubCommand.publish(command);
  }

  // Comprobacion de freno de servicio
  if ((sWrenchEffort->presenceVector & PRESENCE_VECTOR_BRAKE) == PRESENCE_VECTOR_BRAKE) {
    command.id_device = BRAKE;
    command.value = sWrenchEffort->resistiveLinearEffortXPercent;
    instance->pubCommand.publish(command);
  }

  // Liberacion de memoria
  setWrenchEffortMessageDestroy(sWrenchEffort);
}

/** 
 * Método privado que recibe mensajes JAUS de tipo "Set Discrete Devices". 
 * Traduce la información a mensaje ROS (msg_command) y la publica en el topic 
 * correspondiente para la ejecución de comandos de control del vehiculo 
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_set_discrete_devices(OjCmpt cmp, JausMessage msg) {
  SetDiscreteDevicesMessage sDiscreteDevice = setDiscreteDevicesMessageFromJausMessage(msg);
  CITIUS_Control_Communication::msg_command command;

  // Comprobacion de freno de estacionamiento
  if ((sDiscreteDevice->presenceVector & PRESENCE_VECTOR_PARKING_BRAKE) == PRESENCE_VECTOR_PARKING_BRAKE) {
    command.id_device = HANDBRAKE;
    command.value = sDiscreteDevice->parkingBrake;
    instance->pubCommand.publish(command);
  }

  // Comprobacion de marcha
  if ((sDiscreteDevice->presenceVector & PRESENCE_VECTOR_GEAR) == PRESENCE_VECTOR_GEAR) {
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

/** 
 * Método privado que recibe mensajes JAUS de tipo "Set Camera Pose". Traduce la 
 * información a mensaje ROS (msg_ctrlXCamera) y la publica en el topic 
 * correspondiente para la ejecución de comandos de control sobre las cámaras de 
 * apoyo a la conducción
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_set_camera_pose(OjCmpt cmp, JausMessage msg) {
  SetCameraPoseMessage sCameraPose = setCameraPoseMessageFromJausMessage(msg);
  // Camara delantera
  if (sCameraPose->cameraID == FRONT_CAMERA_ID) {

    CITIUS_Control_Communication::msg_ctrlFrontCamera fcCommand;
    fcCommand.isPan = false;
    fcCommand.isTilt = false;
    fcCommand.isZoom = false;

    // Comprobacion de pan
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_PAN) == PRESENCE_VECTOR_PAN) {
      fcCommand.isPan = true;
      fcCommand.pan = sCameraPose->zAngularPositionOrRatePercent;
    }

    // Comprobacion de tilt
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_TILT) == PRESENCE_VECTOR_TILT) {
      fcCommand.isTilt = true;
      fcCommand.tilt = sCameraPose->yAngularPositionOrRatePercent;
    }

    // Comprobacion de zoom
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_ZOOM) == PRESENCE_VECTOR_ZOOM) {
      fcCommand.isZoom = true;
      fcCommand.zoom = sCameraPose->zLinearPositionOrRatePercent;
    }

    instance->pubCtrlFrontCamera.publish(fcCommand);

  }    // Camara trasera
  else if (sCameraPose->cameraID == REAR_CAMERA_ID) {

    CITIUS_Control_Communication::msg_ctrlRearCamera rcCommand;
    rcCommand.isPan = false;
    rcCommand.isTilt = false;
    rcCommand.isZoom = false;

    // Comprobacion de pan
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_PAN) == PRESENCE_VECTOR_PAN) {
      rcCommand.isPan = true;
      rcCommand.pan = sCameraPose->zAngularPositionOrRatePercent;
    }

    // Comprobacion de tilt
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_TILT) == PRESENCE_VECTOR_TILT) {
      rcCommand.isTilt = true;
      rcCommand.tilt = sCameraPose->yAngularPositionOrRatePercent;
    }

    // Comprobacion de zoom
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_ZOOM) == PRESENCE_VECTOR_ZOOM) {
      rcCommand.isZoom = true;
      rcCommand.zoom = sCameraPose->zLinearPositionOrRatePercent;
    }

    instance->pubCtrlRearCamera.publish(rcCommand);

  }

  // Liberacion de memoria
  setCameraPoseMessageDestroy(sCameraPose);
}

/** 
 * Método privado que recibe mensajes JAUS de tipo "Set Signaling Elements". 
 * Traduce la información a mensaje ROS (msg_command) y la publica en el topic 
 * correspondiente para la ejecución de órdenes sobre elementos de señalización 
 * del vehículo
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_set_signaling_elements(OjCmpt cmp, JausMessage msg) {
  SetSignalingElements18Message sSignaling = setSignalingElements18MessageFromJausMessage(msg);
  CITIUS_Control_Communication::msg_command command;

  // Comprobacion de luces de posicion
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSP) == PRESENCE_VECTOR_DIPSP) {
    command.id_device = DIPSP;
    command.value = sSignaling->dipsp;
    instance->pubCommand.publish(command);
  }

  // Comprobacion de luces cortas
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSS) == PRESENCE_VECTOR_DIPSS) {
    command.id_device = DIPSS;
    command.value = sSignaling->dipss;
    instance->pubCommand.publish(command);
  }

  // Comprobacion de luces largas
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSR) == PRESENCE_VECTOR_DIPSR) {
    command.id_device = DIPSR;
    command.value = sSignaling->dipsr;
    instance->pubCommand.publish(command);
  }

  // Comprobacion de intermitente derecha
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_BLINKER_RIGHT) == PRESENCE_VECTOR_BLINKER_RIGHT) {
    command.id_device = BLINKER_RIGHT;
    command.value = sSignaling->blinker_right;
    instance->pubCommand.publish(command);
  }

  // Comprobacion de intermitente izquierda
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_BLINKER_LEFT) == PRESENCE_VECTOR_BLINKER_LEFT) {
    command.id_device = BLINKER_LEFT;
    command.value = sSignaling->blinker_left;
    instance->pubCommand.publish(command);
  }

  // Comprobacion de claxon
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_KLAXON) == PRESENCE_VECTOR_KLAXON) {
    command.id_device = KLAXON;
    command.value = sSignaling->klaxon;
    instance->pubCommand.publish(command);
  }

  // Liberacion de memoria
  setSignalingElements18MessageDestroy(sSignaling);
}

/** 
 * Método privado que recibe mensajes JAUS de tipo "Set Positioner". Obtiene la 
 * información del mensaje y hace uso de los servicios ROS disponibles para 
 * solicitar la ejecución de las ordenes de actuación sobre el posicionador
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_set_positioner(OjCmpt cmp, JausMessage msg) {
  SetPositioner19Message setPositioner = setPositioner19MessageFromJausMessage(msg);

  if (jausByteIsBitSet(setPositioner->presenceVector, JAUS_19_PV_PAN_BIT)) {
    CITIUS_Control_Communication::srv_panAbsolutePosition serviceAbsPan;
    serviceAbsPan.request.panPosition = setPositioner->pan;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientPosPanAbs.call(serviceAbsPan)) {
        numOfAttemps = 10;
        if (serviceAbsPan.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de Pan absoluto a Positioner");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de Pan absoluto a Positioner");
    }
  }

  if (jausByteIsBitSet(setPositioner->presenceVector, JAUS_19_PV_SPIN_VELOCITY_BIT)) {
    CITIUS_Control_Communication::srv_panRate servicePanRate;
    servicePanRate.request.panRate = setPositioner->spin_velocity;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientPosPanRate.call(servicePanRate)) {
        numOfAttemps = 10;
        if (servicePanRate.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de Vel. Pan absoluto a Positioner");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de Vel. Pan a Positioner");
    }
  }

  if (jausByteIsBitSet(setPositioner->presenceVector, JAUS_19_PV_TILT_BIT)) {
    CITIUS_Control_Communication::srv_tiltAbsolutePosition serviceAbsTilt;
    serviceAbsTilt.request.tiltPosition = setPositioner->tilt;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientPosTiltAbs.call(serviceAbsTilt)) {
        numOfAttemps = 10;
        if (serviceAbsTilt.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de Tilt absoluto a Positioner");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de Tilt absoluto a Positioner");
    }
  }

  if (jausByteIsBitSet(setPositioner->presenceVector, JAUS_19_PV_ELEVATION_VELOCITY_BIT)) {
    CITIUS_Control_Communication::srv_tiltRate serviceTiltRate;
    serviceTiltRate.request.tiltRate = setPositioner->elevation_velocity;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientPosTiltRate.call(serviceTiltRate)) {
        numOfAttemps = 10;
        if (serviceTiltRate.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de Tilt Rate a Positioner");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de Tilt Rate a Positioner");
    }
  }

}

/** 
 * Método privado que recibe mensajes de tipo JAUS "Set Day-time Camera". 
 * Obtiene la información del mensaje y hace uso de los servicios ROS 
 * disponibles para solicitar la ejecución de las órdenes de actuación sobre la 
 * cámara TV
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_set_day_time_camera(OjCmpt cmp, JausMessage msg) {
  SetDayTimeCamera21Message setDayCam = setDayTimeCamera21MessageFromJausMessage(msg);

  if (jausByteIsBitSet(setDayCam->presenceVector, JAUS_21_PV_DIRECT_ZOOM_BIT)) {
    CITIUS_Control_Communication::srv_zoomDirect serviceDirectZoom;
    serviceDirectZoom.request.zoomDirect = setDayCam->direct_zoom;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientTVCameraDirectZoom.call(serviceDirectZoom)) {
        numOfAttemps = 10;
        if (serviceDirectZoom.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de Direct Zoom a TV Camera");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de Direct Zoom a TV Camera");
    }
  }

  if (jausByteIsBitSet(setDayCam->presenceVector, JAUS_21_PV_CONTINUOUS_ZOOM_BIT)) {
    CITIUS_Control_Communication::srv_zoomCommand serviceContZoom;
    serviceContZoom.request.zoomCommand = setDayCam->continuous_zoom;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientTVCameraContZoom.call(serviceContZoom)) {
        numOfAttemps = 10;
        if (serviceContZoom.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de Cont. Zoom a TV Camera");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de Cont. Zoom a TV Camera");
    }
  }

  if (jausByteIsBitSet(setDayCam->presenceVector, JAUS_21_PV_FOCUS_BIT)) {
    CITIUS_Control_Communication::srv_focusDirect serviceDirectFocus;
    serviceDirectFocus.request.focusDirect = setDayCam->focus;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientTVCameraFocus.call(serviceDirectFocus)) {
        numOfAttemps = 10;
        if (serviceDirectFocus.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de Focus a TV Camera");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de Focus a TV Camera");
    }
  }

  if (jausByteIsBitSet(setDayCam->presenceVector, JAUS_21_PV_AUTOFOCUS_BIT)) {
    CITIUS_Control_Communication::srv_autofocusMode serviceAutofocus;
    serviceAutofocus.request.autofocus = setDayCam->autofocus;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientTVCameraAutofocus.call(serviceAutofocus)) {
        numOfAttemps = 10;
        if (serviceAutofocus.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de AutoFocus a TV Camera");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de AutoFocus a TV Camera");
    }
  }
}

/** 
 * Método privado que recibe mensajes JAUS de tipo "Set Day-time Camera". 
 * Obtiene la información del mensaje y hace uso de los servicios ROS 
 * disponibles para solicitar la ejecución de las órdenes de actuación sobre la 
 * cámara IR
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_set_night_time_camera(OjCmpt cmp, JausMessage msg) {
  SetNightTimeCamera23Message setNightCam = setNightTimeCamera23MessageFromJausMessage(msg);

  if (jausByteIsBitSet(setNightCam->presenceVector, JAUS_23_PV_ZOOM_BIT)) {
    CITIUS_Control_Communication::srv_dzoom serviceZoom;
    serviceZoom.request.newZoom = setNightCam->zoom;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientIRCameraZoom.call(serviceZoom)) {
        numOfAttemps = 10;
        if (serviceZoom.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de Zoom a IR Camera");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de Zoom a IR Camera");
    }
  }

  if (jausByteIsBitSet(setNightCam->presenceVector, JAUS_23_PV_POLARITY_BIT)) {
    CITIUS_Control_Communication::srv_polarity servicePolarity;
    servicePolarity.request.newPolarity = setNightCam->polarity;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientIRCameraPolarity.call(servicePolarity)) {
        numOfAttemps = 10;
        if (servicePolarity.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Req. de Polarity a IR Camera");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Req. de Polarity a IR Camera");
    }
  }

}

// Componente Platform Sensor

/** 
 * Método privado que recibe mensajes JAUS de tipo "Set Telemeter". Obtiene la 
 * información del mensaje y hace uso de los servicios ROS disponibles para 
 * solicitar la ejecución de las órdenes de actuación sobre el telémetro
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_set_telemeter(OjCmpt cmp, JausMessage msg) {
  SetTelemeter26Message setTelemeterMsg = setTelemeter26MessageFromJausMessage(msg);

  if (setTelemeterMsg->shoot) {
    CITIUS_Control_Communication::srv_shoot serviceTelemeter;
    short numOfAttemps = 0;
    while (numOfAttemps < 5) {
      if (instance->clientShootTel.call(serviceTelemeter)) {
        numOfAttemps = 10;
        if (serviceTelemeter.response.ret == false)
          ROS_INFO("[Control] Communications - Error en el Shoot del telemetro");
      } else {
        numOfAttemps++;
      }
    }
    if (numOfAttemps == 5) {
      //ROS_INFO("[Control] Communications - Error en el Shoot del telemetro");
    }
  }
}

// Componente Velocity State Sensor

/** 
 * Método privado que recibe mensajes JAUS de tipo "Set Travel Speed". Traduce 
 * la información a mensaje ROS (msg_command) y la publica en el topic 
 * correspondiente para la ejecución de comandos de control del vehículo 
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_set_travel_speed(OjCmpt cmp, JausMessage msg) {
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

/** 
 * Método privado que recibe mensajes JAUS de tipo "HeartBeat - Channel State".
 * @todo Recepcion innecesaria en este nivel? 
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_heartbeat_channel_state(OjCmpt cmp, JausMessage msg) {

}

/** 
 * Método privado que recibe mensajes JAUS de tipo "HeartBeat - Position Info".
 * @todo Recepcion innecesaria en este nivel? 
 * @param[in] cmp Componente JAUS emisor
 * @param[in] msg Mensaje JAUS capturado
 */
void RosNode_Communications::fcn_receive_heartbeat_position_info(OjCmpt cmp, JausMessage msg) {

}
