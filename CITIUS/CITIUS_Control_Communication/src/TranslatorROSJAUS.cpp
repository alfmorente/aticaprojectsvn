
/** 
 * @file  TranslatorROSJAUS.cpp
 * @brief Implementación de la clase "TranslatorROSJAUS"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "TranslatorROSJAUS.h"

/** Constructor de la clase*/
TranslatorROSJAUS::TranslatorROSJAUS() {

}

/**
 * Método público que obtiene mensaje JAUS - Report Wrench Effort a partir de 
 * información del vehículo
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] steer Valor de lectura de la dirección
 * @param[in] throttle Valor de lectura del acelerador
 * @param[in] brake Valor de lectura del freno de servicio
 * @return Mensaje JAUS - Report Wrench Effort
 */
JausMessage TranslatorROSJAUS::getJausMsgFromWrenchEffortInfo(JausSubsystemID subDest, JausNodeID nodDest, short steer, short throttle, short brake) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_PRIMITIVE_DRIVER;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  ReportWrenchEffortMessage rwem = reportWrenchEffortMessageCreate();
  rwem->presenceVector = (PRESENCE_VECTOR_BRAKE |
          PRESENCE_VECTOR_THROTTLE |
          PRESENCE_VECTOR_STEER);
  rwem->propulsiveLinearEffortXPercent = throttle;
  rwem->resistiveLinearEffortXPercent = brake;
  rwem->propulsiveRotationalEffortZPercent = steer;
  jausAddressCopy(rwem->destination, jAdd);
  // Generacion de mensaje JUAS global
  jMsg = reportWrenchEffortMessageToJausMessage(rwem);
  reportWrenchEffortMessageDestroy(rwem);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - Report Discrete Devices a partir de
 * información del vehículo 
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] parkingbrake Valor de lectura del freno de estacionamiento
 * @param[in] gear Valor de lectura de la marcha
 * @return Mensaje JAUS - Report Discrete Devices
 */
JausMessage TranslatorROSJAUS::getJausMsgFromDiscreteDeviceInfo(JausSubsystemID subDest, JausNodeID nodDest, bool parkingbrake, short gear) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_PRIMITIVE_DRIVER;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  ReportDiscreteDevicesMessage rddm = reportDiscreteDevicesMessageCreate();
  rddm->presenceVector = (PRESENCE_VECTOR_PARKING_BRAKE | PRESENCE_VECTOR_GEAR);
  rddm->parkingBrake = (JausBoolean) parkingbrake;
  rddm->gear = gear+1;
  //printf("Se va a enviar marcha := %d\n",rddm->gear);
  jausAddressCopy(rddm->destination, jAdd);
  jMsg = reportDiscreteDevicesMessageToJausMessage(rddm);
  reportDiscreteDevicesMessageDestroy(rddm);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - Report Travel Speed a partir de 
 * información del vehículo 
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] speed Valor de lectura de la velocidad de crucero
 * @return Mensaje JAUS - Report Travel Speed
 */
JausMessage TranslatorROSJAUS::getJausMsgFromTravelSpeedInfo(JausSubsystemID subDest, JausNodeID nodDest, short speed) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_VELOCITY_STATE_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  ReportTravelSpeedMessage rtsm = reportTravelSpeedMessageCreate();
  rtsm->speedMps = speed;
  jausAddressCopy(rtsm->destination, jAdd);
  jMsg = reportTravelSpeedMessageToJausMessage(rtsm);
  reportTravelSpeedMessageDestroy(rtsm);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - UGV Info a partir de información 
 * eléctrica del vehículo 
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] motorRPM Valor de lectura de las rpm a las que opera el motor
 * @param[in] motorTemperature Valor de lectura de la temperatura del motor
 * @return Mensaje JAUS - UGV Info
 */
JausMessage TranslatorROSJAUS::getJausMsgFromUGVInfo(JausSubsystemID subDest, JausNodeID nodDest, short motorRPM, short motorTemperature, short idAlarm) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_PRIMITIVE_DRIVER;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  UGVInfo12Message ugvm = ugvInfo12MessageCreate();
  ugvm->presenceVector = (PRESENCE_VECTOR_MOTOR_RPM | 
          PRESENCE_VECTOR_MOTOR_TEMPERATURE | 
          PRESENCE_VECTOR_ALARMS);
  ugvm->motor_rpm = motorRPM;
  ugvm->motor_temperature = motorTemperature;
  ugvm->alarms = idAlarm;
  jausAddressCopy(ugvm->destination, jAdd);
  jMsg = ugvInfo12MessageToJausMessage(ugvm);
  ugvInfo12MessageDestroy(ugvm);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - Report Signaling Elements a partir 
 * de información de señalización del vehículo
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] blinker_left Valor de lectura del intermitente izquierdo
 * @param[in] blinker_right Valor de lectura del intermitente derecho
 * @param[in] dipsp Valor de lectura de las luces de posicion
 * @param[in] dipss Valor de lectura de las luces cortas
 * @param[in] dipsr Valor de lectura de las luces largas
 * @param[in] klaxon Valor de lectura de la bocina
 * @return Mensaje JAUS - Report Signaling Elements
 */
JausMessage TranslatorROSJAUS::getJausMsgFromSignalingInfo(JausSubsystemID subDest, JausNodeID nodDest, bool blinker_left, bool blinker_right, bool dipsp, bool dipss, bool dipsr, bool klaxon) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_VISUAL_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
  rsem->blinker_right = (JausBoolean) blinker_right;
  rsem->blinker_left = (JausBoolean) blinker_left;
  rsem->dipsp = (JausBoolean) dipsp;
  rsem->dipss = (JausBoolean) dipss;
  rsem->dipsr = (JausBoolean) dipsr;
  rsem->klaxon = (JausBoolean) klaxon;
  jausAddressCopy(rsem->destination, jAdd);
  jMsg = reportSignalingElements25MessageToJausMessage(rsem);
  reportSignalingElements25MessageDestroy(rsem);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - UGV Info a partir de información 
 * eléctrica del vehículo
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] bat_level Nivel de batería
 * @param[in] bat_voltage Tensión de batería
 * @param[in] bat_current Intensidad de batería
 * @param[in] bat_temp Temperatura de batería
 * @param[in] alarms Alarmas subsistema electrico
 * @return Mensaje JAUS - UGV Info
 */
JausMessage TranslatorROSJAUS::getJausMsgFromElectricInfo(JausSubsystemID subDest, JausNodeID nodDest, short bat_level, short bat_voltage, short bat_current, short bat_temp, short alarms) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_PRIMITIVE_DRIVER;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  UGVInfo12Message ugvm = ugvInfo12MessageCreate();
  ugvm->presenceVector = PRESENCE_VECTOR_BATTERY_LEVEL |
          PRESENCE_VECTOR_BATTERY_VOLTAGE |
          PRESENCE_VECTOR_BATTERY_CURRENT |
          PRESENCE_VECTOR_BATTERY_TEMPERATURE;
  ugvm->battery_level = bat_level;
  ugvm->battery_voltage = bat_voltage;
  ugvm->battery_current = bat_current;
  ugvm->battery_temperature = bat_temp;
  jausAddressCopy(ugvm->destination, jAdd);
  jMsg = ugvInfo12MessageToJausMessage(ugvm);
  ugvInfo12MessageDestroy(ugvm);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - Report Camera Pose a partir de 
 * información de posicionamiento de las cámaras de apoyo a la conducción
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] id_camera Identificador de cámara
 * @param[in] pan Valor de lectura de PAN
 * @param[in] tilt Valor de lectura de TILT
 * @param[in] zoom Valor de lectura de ZOOM
 * @return  Mensaje JAUS - Report Camera Pose
 */
JausMessage TranslatorROSJAUS::getJausMsgFromCameraInfo(JausSubsystemID subDest, JausNodeID nodDest, CameraID id_camera, short pan, short tilt, short zoom) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_VISUAL_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  ReportCameraPoseMessage rcpm = reportCameraPoseMessageCreate();
  rcpm->cameraID = id_camera;
  rcpm->presenceVector = (PRESENCE_VECTOR_CURRENT_PAN | PRESENCE_VECTOR_CURRENT_TILT | PRESENCE_VECTOR_CURRENT_ZOOM);
  rcpm->xCameraAxisDirectionCosineZ = (pan * CONV_JAUS_PANTILT);
  rcpm->xCameraAxisDirectionCosineY = (tilt * CONV_JAUS_PANTILT);
  rcpm->zCameraOriginMeters = (zoom * CONV_JAUS_ZOOM);
  jausAddressCopy(rcpm->destination, jAdd);
  jMsg = reportCameraPoseMessageToJausMessage(rcpm);
  reportCameraPoseMessageDestroy(rcpm);
  jausAddressDestroy(jAdd);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - Report Night-time Camera a partir 
 * de información de la cámara IR
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] zoom Valor de lectura de ZOOM
 * @param[in] polarity Valor de lectura de POLARIDAD
 * @return Mensaje JAUS - Report Night-time Camera
 */
JausMessage TranslatorROSJAUS::getJausMsgFromIRCameraInfo(JausSubsystemID subDest, JausNodeID nodDest, short zoom, short polarity) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_VISUAL_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  ReportNightTimeCamera24Message ircm = reportNightTimeCamera24MessageCreate();
  ircm->active_zoom = zoom;
  if (polarity == 0) {
    ircm->active_polarity = JAUS_FALSE;
  } else if (polarity == 1) {
    ircm->active_polarity = JAUS_TRUE;
  }
  jausAddressCopy(ircm->destination, jAdd);
  jMsg = reportNightTimeCamera24MessageToJausMessage(ircm);
  reportNightTimeCamera24MessageDestroy(ircm);
  jausAddressDestroy(jAdd);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - Report Telemeter a partir de 
 * información del telémetro
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] ecs Valor de lectura de los ecos encontrados
 * @return Mensaje JAUS - Report Telemeter
 */
JausMessage TranslatorROSJAUS::getJausMsgFromTelemeterInfo(JausSubsystemID subDest, JausNodeID nodDest, short *ecs) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_PLATFORM_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  ReportTelemeter27Message tim = reportTelemeter27MessageCreate();
  for (int i = 0; i < 5; i++) {
    tim->echoes[i] = ecs[i];
  }
  jausAddressCopy(tim->destination, jAdd);
  jMsg = reportTelemeter27MessageToJausMessage(tim);
  reportTelemeter27MessageDestroy(tim);
  jausAddressDestroy(jAdd);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - Report Day-time Camera a partir de 
 * información de la cámara TV
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] zoom Valor de lectura de ZOOM
 * @param[in] focus Valor de lectura de FOCO
 * @param[in] autofocus Valor de lectura de AUTOFOCO
 * @return Mensaje JAUS - Report Day-time Camera
 */
JausMessage TranslatorROSJAUS::getJausMsgFromTVCamera(JausSubsystemID subDest, JausNodeID nodDest, short zoom, short focus, bool autofocus) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_VISUAL_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  ReportDayTimeCamera22Message dtcm = reportDayTimeCamera22MessageCreate();
  dtcm->active_zoom = zoom;
  dtcm->active_focus = focus;
  dtcm->active_autofocus = (JausBoolean) autofocus;
  jausAddressCopy(dtcm->destination, jAdd);
  jMsg = reportDayTimeCamera22MessageToJausMessage(dtcm);
  reportDayTimeCamera22MessageDestroy(dtcm);
  jausAddressDestroy(jAdd);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - Report Positioner a partir de 
 * información del posicionador
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] pan Valor de lectura de PAN
 * @param[in] tilt Valor de lectura de TILT
 * @return Mensaje JAUS - Report Positioner
 */
JausMessage TranslatorROSJAUS::getJausMsgFromPositioner(JausSubsystemID subDest, JausNodeID nodDest, short pan, short tilt) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_VISUAL_SENSOR;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  ReportPositioner20Message posm = reportPositioner20MessageCreate();
  posm->active_pan = pan;
  posm->active_tilt = tilt;
  jausAddressCopy(posm->destination, jAdd);
  jMsg = reportPositioner20MessageToJausMessage(posm);
  reportPositioner20MessageDestroy(posm);
  jausAddressDestroy(jAdd);
  return jMsg;
}

/**
 * Método público que obtiene mensaje JAUS - UGV Info a partir de una alarma
 * @param[in] subDest Subsistema de destino
 * @param[in] nodDest Nodo de destino
 * @param[in] id_alarm Identificador de la alarma a enviar
 * @return Mensaje JAUS - UGV Info
 */
JausMessage TranslatorROSJAUS::getJausMsgFromAlarm(JausSubsystemID subDest, JausNodeID nodDest, short id_alarm) {
  JausMessage jMsg = NULL;
  JausAddress jAdd = jausAddressCreate();
  jAdd->subsystem = subDest;
  jAdd->node = nodDest;
  jAdd->component = JAUS_PRIMITIVE_DRIVER;
  jAdd->instance = JAUS_DESTINANTION_INSTANCE;
  UGVInfo12Message ugvm = ugvInfo12MessageCreate();
  ugvm->presenceVector = PRESENCE_VECTOR_ALARMS;
  ugvm->alarms = id_alarm;
  jausAddressCopy(ugvm->destination, jAdd);
  jMsg = ugvInfo12MessageToJausMessage(ugvm);
  ugvInfo12MessageDestroy(ugvm);
  jausAddressDestroy(jAdd);
  return jMsg;
}

