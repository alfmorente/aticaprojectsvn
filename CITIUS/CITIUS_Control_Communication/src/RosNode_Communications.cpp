
/** 
 * @file  RosNode_Communications.cpp
 * @brief Implementación de la clase "RosNode_Communications"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include <cmath>

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
  subsystemController = JAUS_SUBSYSTEM_MYC; // MyC
  nodeController = JAUS_NODE_CONTROL; // Control
  nodeStatus = NODESTATUS_INIT;
  // Inicializador de selectores de camaras
  currentDrivingCamera = FRONT_CAMERA_ID;
  currentObservationCamera = TV_CAMERA;
  hbPosition.latitude = 0;
  hbPosition.longitude = 0;
  hbPosition.altitude = 0;
  hbPosition.heading = 0;
  hbPosition.speed = 0;
}

/**
 * Destructor de la clase
 */
RosNode_Communications::~RosNode_Communications() {

}

/** 
 * Método público que inicia los artefactos ROS que la clase contiene en sus 
 * atributos 
 */
void RosNode_Communications::initROS() {
  ros::NodeHandle nh;
  // Inicializacion de publicadores
  pubCommand = nh.advertise<CITIUS_Control_Communication::msg_command>("command", 1000);
  pubElectricCommand = nh.advertise<CITIUS_Control_Communication::msg_electricCommand>("electricCommand", 1000);
  pubCtrlFrontCamera = nh.advertise<CITIUS_Control_Communication::msg_ctrlFrontCamera>("ctrlFrontCamera", 1000);
  pubCtrlRearCamera = nh.advertise<CITIUS_Control_Communication::msg_ctrlRearCamera>("ctrlRearCamera", 1000);
  // Inicializacion de suscriptores
  //          Subsistema de control
  subsElectricInfo = nh.subscribe("electricInfo", 1000, &RosNode_Communications::fnc_subs_electricInfo, this);
  subsVehicleInfo = nh.subscribe("vehicleInfo", 1000, &RosNode_Communications::fnc_subs_vehicleInfo, this);
  subsFrontCameraInfo = nh.subscribe("frontCameraInfo", 1000, &RosNode_Communications::fnc_subs_frontCameraInfo, this);
  subsRearCameraInfo = nh.subscribe("rearCameraInfo", 1000, &RosNode_Communications::fnc_subs_rearCameraInfo, this);
  subsPosOriInfo = nh.subscribe("posOriInfo", 1000, &RosNode_Communications::fnc_subs_posOriInfo, this);
  subsLastExec = nh.subscribe("lastExec", 1000, &RosNode_Communications::fcn_subs_lastExec, this);
  
  //          Subsistema de payload de observacion
  subsIRCameraInfo = nh.subscribe("IRInformation", 1000, &RosNode_Communications::fcn_subs_irCameraInfo, this);
  subsTelemeterInfo = nh.subscribe("LRFEchoesFound", 1000, &RosNode_Communications::fcn_subs_telemeterInfo, this);
  subsTVCameraInfo = nh.subscribe("TVInformation", 1000, &RosNode_Communications::fcn_subs_tvCameraInfo, this);
  subsPositionerInfo = nh.subscribe("PanTiltPosition", 1000, &RosNode_Communications::fcn_subs_positionerInfo, this);

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

  try {
    configData = new FileLoader("/home/ugv/catkin_ws/src/CITIUS_Control_Communication/bin/nodeManager.conf");
    handler = new JausHandler();
    nm = new NodeManager(this->configData, this->handler);
  } catch (...) {
    ROS_INFO("[Control] Communications - No se ha podido inicializar JAUS");
  }

  //Creacion de componentes
  // Mission Spooler
  missionSpoolerComponent = ojCmptCreate((char *) "Mission Spooler", JAUS_MISSION_SPOOLER, 1);
  if (missionSpoolerComponent == NULL) {
    ROS_INFO("[Control] Communication - No se ha podido crear el componente MISSION SPOOLER");
    exit(0);
  } else {
    ojCmptAddServiceOutputMessage(missionSpoolerComponent, JAUS_MISSION_SPOOLER, JAUS_REPORT_MISSION_STATUS, 0xFF);
    ojCmptAddServiceInputMessage(missionSpoolerComponent, JAUS_MISSION_SPOOLER, JAUS_RUN_MISSION, 0xFF);
    ojCmptSetMessageCallback(missionSpoolerComponent, JAUS_RUN_MISSION, fcn_receive_run_mission);
  }

  // Primitive Driver
  primitiveDriverComponent = ojCmptCreate((char *) "Primitive Driver", JAUS_PRIMITIVE_DRIVER, 1);
  if (primitiveDriverComponent == NULL) {
    ROS_INFO("[Control] Communication - No se ha podido crear el componente PRIMITIVE DRIVER");
    exit(0);
  } else {
    ojCmptAddServiceOutputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_WRENCH_EFFORT, 0xFF);
    ojCmptAddServiceOutputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_REPORT_DISCRETE_DEVICES, 0xFF);
    ojCmptAddServiceOutputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_UGV_INFO_12, 0xFF);
    ojCmptAddServiceInputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_SET_WRENCH_EFFORT, 0xFF);
    ojCmptAddServiceInputMessage(primitiveDriverComponent, JAUS_PRIMITIVE_DRIVER, JAUS_SET_DISCRETE_DEVICES, 0xFF);
    ojCmptSetMessageCallback(primitiveDriverComponent, JAUS_SET_WRENCH_EFFORT, fcn_receive_set_wrench_effort);
    ojCmptSetMessageCallback(primitiveDriverComponent, JAUS_SET_DISCRETE_DEVICES, fcn_receive_set_discrete_devices);
  }

  // Visual Sensor
  visualSensorComponent = ojCmptCreate((char *) "Visual Sensor", JAUS_VISUAL_SENSOR, 1);
  if (visualSensorComponent == NULL) {
    ROS_INFO("[Control] Communication - No se ha podido crear el componente VISUAL SENSOR");
    exit(0);
  } else {
    ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_CAMERA_POSE, 0xFF);
    ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_SIGNALING_ELEMENTS_25, 0xFF);
    ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_POSITIONER_20, 0xFF);
    ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_DAY_TIME_CAMERA_22, 0xFF);
    ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_REPORT_NIGHT_TIME_CAMERA_24, 0xFF);
    ojCmptAddServiceOutputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SELECT_CAMERA, 0xFF);
    ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_CAMERA_POSE, 0xFF);
    ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_SIGNALING_ELEMENTS_18, 0xFF);
    ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_POSITIONER_19, 0xFF);
    ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_DAY_TIME_CAMERA_21, 0xFF);
    ojCmptAddServiceInputMessage(visualSensorComponent, JAUS_VISUAL_SENSOR, JAUS_SET_NIGHT_TIME_CAMERA_23, 0xFF);
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
  } else {
    ojCmptAddServiceOutputMessage(platformSensorComponent, JAUS_PLATFORM_SENSOR, JAUS_REPORT_TELEMETER_27, 0xFF);
    ojCmptAddServiceInputMessage(platformSensorComponent, JAUS_PLATFORM_SENSOR, JAUS_SET_TELEMETER_26, 0xFF);
    ojCmptSetMessageCallback(platformSensorComponent, JAUS_SET_TELEMETER_26, fcn_receive_set_telemeter);

  }

  // Velocity State Sensor
  velocityStateSensorComponent = ojCmptCreate((char *) "Velocity State Sensor", JAUS_VELOCITY_STATE_SENSOR, 1);
  if (velocityStateSensorComponent == NULL) {
    ROS_INFO("[Control] Communication - No se ha podido crear el componente VELOCITY STATE SENSOR");
    exit(0);
  } else {
    ojCmptAddServiceOutputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_REPORT_TRAVEL_SPEED, 0xFF);
    ojCmptAddServiceOutputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_REPORT_VELOCITY_STATE, 0xFF);
    ojCmptAddServiceInputMessage(velocityStateSensorComponent, JAUS_VELOCITY_STATE_SENSOR, JAUS_SET_TRAVEL_SPEED, 0xFF);
    ojCmptSetMessageCallback(velocityStateSensorComponent, JAUS_SET_TRAVEL_SPEED, fcn_receive_set_travel_speed);
  }

  // Global Pose Sensor
  globalPoseSensorComponent = ojCmptCreate((char *) "Global Pose Sensor", JAUS_GLOBAL_POSE_SENSOR, 1);
  if (globalPoseSensorComponent == NULL) {
    ROS_INFO("[Control] Communication - No se ha podido crear el componente GLOBAL POSE SENSOR");
    exit(0);
  } else {
    ojCmptAddServiceOutputMessage(globalPoseSensorComponent, JAUS_GLOBAL_POSE_SENSOR, JAUS_REPORT_GLOBAL_POSE, 0xFF);
    ojCmptAddServiceOutputMessage(globalPoseSensorComponent, JAUS_GLOBAL_POSE_SENSOR, JAUS_ADITIONAL_GPSINS_INFO_4, 0xFF);
  }

  // HeartBeat Information
  heartBeatInformationComponent = ojCmptCreate((char *) "HeartBeat Information", JAUS_HEARTBEAT_INFORMATION, 1);
  if (heartBeatInformationComponent == NULL) {
    ROS_INFO("[Control] Communication - No se ha podido crear el componente HEARTBEAT INFORMATION");
    exit(0);
  } else {
    ojCmptAddServiceOutputMessage(heartBeatInformationComponent, JAUS_HEARTBEAT_INFORMATION, JAUS_HEARTBEAT_POSITION_INFO_17, 0xFF);
  }

  // Run de componentes
  ojCmptRun(missionSpoolerComponent);
  ojCmptRun(primitiveDriverComponent);
  ojCmptRun(visualSensorComponent);
  ojCmptRun(platformSensorComponent);
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
  ojCmptDestroy(velocityStateSensorComponent);
  ojCmptDestroy(globalPoseSensorComponent);
  ojCmptDestroy(heartBeatInformationComponent);
}

/**
 * Método público que espera a la disponibilidad de uno de los controladores
 * (MyC o Tablet) en la arquitectura JAUS
 * @return 
 */
bool RosNode_Communications::isControllerAvailable(){
  if(handler->isMyCAvailable()){
    ROS_INFO("[Control] Communications - Controlador: Mando y Control");
    subsystemController = JAUS_SUBSYSTEM_MYC;
    nodeController = JAUS_NODE_CONTROL;
    return true;
  }else{
    if(handler->isTabletAvailable()){
      ROS_INFO("[Control] Communications - Controlador: Tablet");
      subsystemController = JAUS_SUBSYSTEM_UGV;
      nodeController = JAUS_NODE_TABLET;
      return true;
    }
  }
  return false;
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
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = instance->subsystemController;
  jAdd->node = instance->nodeController;
  jAdd->component = JAUS_MISSION_SPOOLER;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
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
  jMsg = reportMissionStatusMessageToJausMessage(rmsm);
  if (jMsg != NULL) {
    ojCmptSendMessage(instance->missionSpoolerComponent, jMsg);
  } else {
    ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion electrica de vehiculo");
  }
  reportMissionStatusMessageDestroy(rmsm);
  jausAddressDestroy(jAdd);
  jausMessageDestroy(jMsg);
}

/**
 * Método público que envía información sobre la cámara de la que debe enviar el
 * streaming el nodo JAUS de Communication Management
 */
void RosNode_Communications::informCameraToStream(){
  int status;
  ros::NodeHandle nh;
  nh.getParam("vehicleStatus", status);
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = JAUS_SUBSYSTEM_UGV;
  jAdd->node = JAUS_NODE_COMM_MNG;
  jAdd->component = JAUS_VISUAL_SENSOR;
  SelectCameraMessage scm = selectCameraMessageCreate();
  if (status == OPERATION_MODE_OBSERVACION) {
    scm->cameraID = currentObservationCamera;
  } else {
    scm->cameraID = currentDrivingCamera;
  } 
  jausAddressCopy(scm->destination, jAdd);
  jMsg = selectCameraMessageToJausMessage(scm);
  if (jMsg != NULL) {
    ojCmptSendMessage(instance->visualSensorComponent, jMsg);
  } else {
    ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion electrica de vehiculo");
  }
  selectCameraMessageDestroy(scm);
  jausAddressDestroy(jAdd);
  jausMessageDestroy(jMsg);
}

/**
 * Método público que envía información de heartbeat (mensaje JAUS Heartbeat - 
 * Position Info) a los nodos JAUS de Communication Management de todos los
 * subsistemas
 */
void RosNode_Communications::informHeartbeatPositionInfo() {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = JAUS_SUBSYSTEM_UGV;
  jAdd->node = JAUS_NODE_COMM_MNG;
  jAdd->component = JAUS_HEARTBEAT_INFORMATION;
  HeartbeatPositionInfo17Message hpi = heartbeatPositionInfo17MessageCreate();
  hpi->latitude = hbPosition.latitude;
  hpi->longitude = hbPosition.longitude;
  hpi->altitude = hbPosition.altitude;
  hpi->heading = hbPosition.heading;
  hpi->speed = hbPosition.speed;
  // Hacia UGV
  jausAddressCopy(hpi->destination, jAdd);
  jMsg = heartbeatPositionInfo17MessageToJausMessage(hpi);
  if (jMsg != NULL) {
    ojCmptSendMessage(instance->heartBeatInformationComponent, jMsg);
  } else {
    ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion heartbeat hacia UGV");
  }
  // Hacia MyC
  jAdd->subsystem = JAUS_SUBSYSTEM_MYC;
  jausAddressCopy(hpi->destination, jAdd);
  jMsg = heartbeatPositionInfo17MessageToJausMessage(hpi);
  if (jMsg != NULL) {
    ojCmptSendMessage(instance->heartBeatInformationComponent, jMsg);
  } else {
    ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion heartbeat hacia MyC");
  }
  // Hacia USV
  jAdd->subsystem = JAUS_SUBSYSTEM_USV;
  jausAddressCopy(hpi->destination, jAdd);
  jMsg = heartbeatPositionInfo17MessageToJausMessage(hpi);
  if (jMsg != NULL) {
    ojCmptSendMessage(instance->heartBeatInformationComponent, jMsg);
  } else {
    ROS_INFO("[Control] Communications - No se ha posdido generar mensaje JAUS con informacion heartbeat hacia USV");
  }
  
  heartbeatPositionInfo17MessageDestroy(hpi);
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
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  JausMessage jMsg = translator->getJausMsgFromCameraInfo(this->subsystemController, this->nodeController, FRONT_CAMERA_ID, msg.pan, msg.tilt, msg.zoom);
  if (jMsg != NULL) {
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
  }
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
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  JausMessage jMsg = translator->getJausMsgFromCameraInfo(this->subsystemController, this->nodeController, REAR_CAMERA_ID, msg.pan, msg.tilt, msg.zoom);
  if (jMsg != NULL) {
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
  }
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
  if(msg.gear == 2) 
    instance->currentDrivingCamera = REAR_CAMERA_ID;
  else
    instance->currentDrivingCamera = FRONT_CAMERA_ID;
  
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  JausMessage jMsg = translator->getJausMsgFromWrenchEffortInfo(this->subsystemController, this->nodeController, msg.steering, msg.thottle, msg.brake);
  if (jMsg != NULL) {
    ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
  }
  // Report Discrete Devices
  jMsg = translator->getJausMsgFromDiscreteDeviceInfo(this->subsystemController, this->nodeController, msg.parkingBrake, msg.gear);
  if (jMsg != NULL) {
    ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
  }
  // Report Travel Speed
  jMsg = translator->getJausMsgFromTravelSpeedInfo(this->subsystemController, this->nodeController, msg.speed);
  if (jMsg != NULL) {
    ojCmptSendMessage(this->velocityStateSensorComponent, jMsg);
  }
  // UGV Info
  jMsg = translator->getJausMsgFromUGVInfo(this->subsystemController, this->nodeController, msg.motorRPM, msg.motorTemperature, msg.alarms);
  if (jMsg != NULL) {
    ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
  }
  // Report Signaling Elements
  if (msg.lights) {
    jMsg = translator->getJausMsgFromSignalingInfo(this->subsystemController, this->nodeController, msg.blinkerLeft, msg.blinkerRight, msg.dipsp, msg.dipss, msg.dipsr, msg.klaxon);
    if (jMsg != NULL) {
      ojCmptSendMessage(this->visualSensorComponent, jMsg);
    }
  }
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información del módulo de 
 * gestión eléctrica del vehículo. Traduce la información a JAUS (UGV Info) y la 
 * envía al controlador
 * @param[in] msg Mensaje ROS con información del modulo de conducción
 */
void RosNode_Communications::fnc_subs_electricInfo(CITIUS_Control_Communication::msg_electricInfo msg) {
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  JausMessage jMsg = translator->getJausMsgFromElectricInfo(this->subsystemController, this->nodeController, msg.battery_level, msg.battery_voltage, msg.battery_current, msg.battery_temperature, msg.supply_alarms);
  // decomentar
  if (jMsg != NULL) {
    ojCmptSendMessage(this->primitiveDriverComponent, jMsg);
  }
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
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subsystemController;
  jAdd->node = nodeController;
  jAdd->component = JAUS_GLOBAL_POSE_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  // Report Global Pose
  ReportGlobalPoseMessage rgpm = reportGlobalPoseMessageCreate();
  rgpm->presenceVector = 0x0077;
  rgpm->latitudeDegrees = msg.latitude;
  hbPosition.latitude = msg.latitude;
  rgpm->longitudeDegrees = msg.longitude;
  hbPosition.longitude = msg.longitude;
  rgpm->attitudeRmsRadians = msg.altitude;
  hbPosition.altitude = msg.altitude;
  rgpm->rollRadians = msg.roll;
  rgpm->pitchRadians = msg.pitch;
  rgpm->yawRadians = msg.yaw;
  hbPosition.heading = msg.yaw;
  jausAddressCopy(rgpm->destination, jAdd);
  jMsg = reportGlobalPoseMessageToJausMessage(rgpm);
  reportGlobalPoseMessageDestroy(rgpm);
  if (jMsg != NULL) {
    ojCmptSendMessage(globalPoseSensorComponent, jMsg);
  }
  // Report Velocity State
  ReportVelocityStateMessage rvsm = reportVelocityStateMessageCreate();
  rvsm->presenceVector = 0x0007;
  rvsm->velocityXMps = msg.velX;
  rvsm->velocityYMps = msg.velY;
  rvsm->velocityZMps = msg.velZ;
  hbPosition.speed = sqrt(pow(msg.velX,2)+pow(msg.velY,2)+pow(msg.velZ,2));
  jAdd->component = JAUS_VELOCITY_STATE_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  jausAddressCopy(rvsm->destination, jAdd);
  jMsg = reportVelocityStateMessageToJausMessage(rvsm);
  reportVelocityStateMessageDestroy(rvsm);
  if (jMsg != NULL) {
    ojCmptSendMessage(velocityStateSensorComponent, jMsg);
  }

  // Additional GPS/INS Info
  AditionalGPSINSInfo4Message agim = aditionalGPSINSInfo4MessageCreate();
  agim->presenceVector = 0x0007;
  agim->longitudinal_acc = msg.accX;
  agim->lateral_acc = msg.accY;
  agim->vertical_acc = msg.accZ;
  // Estado IMU y GPS
  // TODO!!!!!!!!!!!!!!!!!!!!!!!!
  jAdd->component = JAUS_GLOBAL_POSE_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  jausAddressCopy(agim->destination, jAdd);
  jMsg = aditionalGPSINSInfo4MessageToJausMessage(agim);
  aditionalGPSINSInfo4MessageDestroy(agim);
  if (jMsg != NULL) {
    ojCmptSendMessage(velocityStateSensorComponent, jMsg);
  }
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
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  JausMessage jMsg = translator->getJausMsgFromIRCameraInfo(this->subsystemController, this->nodeController, msg.currentDZoom, msg.currentPolarity);
  if (jMsg != NULL) {
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
  }
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información recopilada del 
 * telémetro. Traduce la información a JAUS (Report Telemeter) y la envía al 
 * controlador
 * @param[in] msg Mensaje ROS con información de telemetro
 */
void RosNode_Communications::fcn_subs_telemeterInfo(CITIUS_Control_Communication::msg_echoesFound msg) {
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  JausMessage jMsg = translator->getJausMsgFromTelemeterInfo(this->subsystemController, this->nodeController, msg.echoesFound.c_array());
  if (jMsg != NULL) {
    ojCmptSendMessage(this->platformSensorComponent, jMsg);
  }
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información recopilada de la 
 * cámara TV. Traduce la información a JAUS (Report Day-time Camera) y la envía 
 * al controlador
 * @param[in] msg Mensaje ROS con información de camara TV
 */
void RosNode_Communications::fcn_subs_tvCameraInfo(CITIUS_Control_Communication::msg_tvinfo msg) {
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  JausMessage jMsg = translator->getJausMsgFromTVCamera(this->subsystemController, this->nodeController, msg.currentZoom, msg.currentFocus, msg.autofocusMode);
  if (jMsg != NULL) {
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
  }
  jausMessageDestroy(jMsg);
}

/** 
 * Método privado consumidor de topic asociado a información recopilada del 
 * posicionador. Traduce la información a JAUS (Report Positioner) y la envía al 
 * controlador
 * @param[in] msg Mensaje ROS con información del posicionador
 */
void RosNode_Communications::fcn_subs_positionerInfo(CITIUS_Control_Communication::msg_panTiltPosition msg) {
  TranslatorROSJAUS *translator = new TranslatorROSJAUS();
  JausMessage jMsg = translator->getJausMsgFromPositioner(this->subsystemController, this->nodeController, msg.panPosition, msg.tiltPosition);
  if (jMsg != NULL) {
    ojCmptSendMessage(this->visualSensorComponent, jMsg);
  }
  jausMessageDestroy(jMsg);
}

/**
 * Método privado consumidor del topic asociado a información sobre mal apagado
 * del sistema de control del vehículo durante la ejecución anterior.
 * @param[in] msg Mensaje ROS con información sobre mal apagado en anterior
 * ejecución
 */
void RosNode_Communications::fcn_subs_lastExec(CITIUS_Control_Communication::msg_lastExec msg) {
  if (msg.badExec) {
    TranslatorROSJAUS *translator = new TranslatorROSJAUS();
    JausMessage jMsg = translator->getJausMsgFromAlarm(this->subsystemController, this->nodeController, ID_ALARMS_WRONG_TURN_OFF);
    if (jMsg != NULL) {
      ojCmptSendMessage(this->visualSensorComponent, jMsg);
    }
    jausMessageDestroy(jMsg);
  }
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
    printf("Run Mission\n");
    RunMissionMessage rMission = runMissionMessageFromJausMessage(msg);
  CITIUS_Control_Communication::srv_vehicleStatus vehicleStatus;
  short tempStatus = rMission->missionId;
  printf("Run Mission recibido :: Estado:= %d\n",tempStatus);
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
  printf("Set Wrench Effort\n");
    SetWrenchEffortMessage sWrenchEffort = setWrenchEffortMessageFromJausMessage(msg);
  CITIUS_Control_Communication::msg_command command;
  if ((sWrenchEffort->presenceVector & PRESENCE_VECTOR_STEER) == PRESENCE_VECTOR_STEER) {
    command.id_device = STEERING;
    command.value = sWrenchEffort->propulsiveRotationalEffortZPercent;
    instance->pubCommand.publish(command);
  }
  if ((sWrenchEffort->presenceVector & PRESENCE_VECTOR_THROTTLE) == PRESENCE_VECTOR_THROTTLE) {
    command.id_device = THROTTLE;
    command.value = sWrenchEffort->propulsiveLinearEffortXPercent;
    instance->pubCommand.publish(command);
  }
  if ((sWrenchEffort->presenceVector & PRESENCE_VECTOR_BRAKE) == PRESENCE_VECTOR_BRAKE) {
    command.id_device = BRAKE;
    command.value = sWrenchEffort->resistiveLinearEffortXPercent;
    instance->pubCommand.publish(command);
  }
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
  printf("Set Discrete Device\n");
    SetDiscreteDevicesMessage sDiscreteDevice = setDiscreteDevicesMessageFromJausMessage(msg);
  CITIUS_Control_Communication::msg_command command;
  if ((sDiscreteDevice->presenceVector & PRESENCE_VECTOR_PARKING_BRAKE) == PRESENCE_VECTOR_PARKING_BRAKE) {
    command.id_device = HANDBRAKE;
    command.value = sDiscreteDevice->parkingBrake;
    instance->pubCommand.publish(command);
    /*if(sDiscreteDevice->parkingBrake)
        printf("Handbrake ON\n");
    else
        printf("Handbrake OFF\n");*/
  }
  if ((sDiscreteDevice->presenceVector & PRESENCE_VECTOR_GEAR) == PRESENCE_VECTOR_GEAR) {
    command.id_device = GEAR;
    command.value = sDiscreteDevice->gear-1;
    //printf("Recibida marcha := %d\n",command.value+1);
    instance->pubCommand.publish(command);
  }
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
  printf("Set Camera Pose\n");
    SetCameraPoseMessage sCameraPose = setCameraPoseMessageFromJausMessage(msg);
  
  // Camara delantera
  if (sCameraPose->cameraID == FRONT_CAMERA_ID) {
    CITIUS_Control_Communication::msg_ctrlFrontCamera fcCommand;
    fcCommand.isPan = false;
    fcCommand.isTilt = false;
    fcCommand.isZoom = false;
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_PAN) == PRESENCE_VECTOR_PAN) {
      fcCommand.isPan = true;
      fcCommand.pan = sCameraPose->zAngularPositionOrRatePercent;
    }
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_TILT) == PRESENCE_VECTOR_TILT) {
      fcCommand.isTilt = true;
      fcCommand.tilt = sCameraPose->yAngularPositionOrRatePercent;
    }
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_ZOOM) == PRESENCE_VECTOR_ZOOM) {
      fcCommand.isZoom = true;
      fcCommand.zoom = sCameraPose->zLinearPositionOrRatePercent;
    }
    instance->pubCtrlFrontCamera.publish(fcCommand);
  }
  // Camara trasera
  else if (sCameraPose->cameraID == REAR_CAMERA_ID) {
    CITIUS_Control_Communication::msg_ctrlRearCamera rcCommand;
    rcCommand.isPan = false;
    rcCommand.isTilt = false;
    rcCommand.isZoom = false;
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_PAN) == PRESENCE_VECTOR_PAN) {
      rcCommand.isPan = true;
      rcCommand.pan = sCameraPose->zAngularPositionOrRatePercent;
    }
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_TILT) == PRESENCE_VECTOR_TILT) {
      rcCommand.isTilt = true;
      rcCommand.tilt = sCameraPose->yAngularPositionOrRatePercent;
    }
    if ((sCameraPose->presenceVector & PRESENCE_VECTOR_ZOOM) == PRESENCE_VECTOR_ZOOM) {
      rcCommand.isZoom = true;
      rcCommand.zoom = sCameraPose->zLinearPositionOrRatePercent;
    }
    instance->pubCtrlRearCamera.publish(rcCommand);
  }
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
    printf("Recibido Signaling\n");
  SetSignalingElements18Message sSignaling = setSignalingElements18MessageFromJausMessage(msg);
  CITIUS_Control_Communication::msg_command command;
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSP) == PRESENCE_VECTOR_DIPSP) {
    command.id_device = DIPSP;
    command.value = sSignaling->dipsp;
    instance->pubCommand.publish(command);
  }
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSS) == PRESENCE_VECTOR_DIPSS) {
    command.id_device = DIPSS;
    command.value = sSignaling->dipss;
    instance->pubCommand.publish(command);
  }
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_DIPSR) == PRESENCE_VECTOR_DIPSR) {
    command.id_device = DIPSR;
    command.value = sSignaling->dipsr;
    instance->pubCommand.publish(command);
  }
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_BLINKER_RIGHT) == PRESENCE_VECTOR_BLINKER_RIGHT) {
    command.id_device = BLINKER_RIGHT;
    command.value = sSignaling->blinker_right;
    instance->pubCommand.publish(command);
  }
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_BLINKER_LEFT) == PRESENCE_VECTOR_BLINKER_LEFT) {
    command.id_device = BLINKER_LEFT;
    command.value = sSignaling->blinker_left;
    instance->pubCommand.publish(command);
  }
  if ((sSignaling->presenceVector & PRESENCE_VECTOR_KLAXON) == PRESENCE_VECTOR_KLAXON) {
    command.id_device = KLAXON;
    command.value = sSignaling->klaxon;
    instance->pubCommand.publish(command);
  }
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
  printf("Set Positioner\n");
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
      ROS_INFO("[Control] Communications - Error en el Req. de Pan absoluto a Positioner");
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
      ROS_INFO("[Control] Communications - Error en el Req. de Vel. Pan a Positioner");
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
      ROS_INFO("[Control] Communications - Error en el Req. de Tilt absoluto a Positioner");
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
      ROS_INFO("[Control] Communications - Error en el Req. de Tilt Rate a Positioner");
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
  printf("Set Day-time camera\n");
    instance->currentObservationCamera = TV_CAMERA;
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
      ROS_INFO("[Control] Communications - Error en el Req. de Direct Zoom a TV Camera");
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
      ROS_INFO("[Control] Communications - Error en el Req. de Cont. Zoom a TV Camera");
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
      ROS_INFO("[Control] Communications - Error en el Req. de Focus a TV Camera");
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
      ROS_INFO("[Control] Communications - Error en el Req. de AutoFocus a TV Camera");
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
 printf("Set night-time camera\n");
    instance->currentObservationCamera = IR_CAMERA;
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
      ROS_INFO("[Control] Communications - Error en el Req. de Zoom a IR Camera");
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
      ROS_INFO("[Control] Communications - Error en el Req. de Polarity a IR Camera");
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
 printf("Set Telemeter\n");
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
      ROS_INFO("[Control] Communications - Error en el Shoot del telemetro");
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
  printf("Set Travel Speed\n");
    SetTravelSpeedMessage sTravelSpeed = setTravelSpeedMessageFromJausMessage(msg);
  CITIUS_Control_Communication::msg_command command;
  command.id_device = CRUISING_SPEED;
  command.value = sTravelSpeed->speedMps;
  instance->pubCommand.publish(command);
  setTravelSpeedMessageDestroy(sTravelSpeed);
}

