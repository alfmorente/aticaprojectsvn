#include "TranslatorROSJAUS.h"
#include "constant.h"
#include "message/experimental/ugvInfo12Message.h"
#include "message/experimental/reportSignalingElements25Message.h"

/**
 * @brief Constructor de la clase TranslatorROSJAUS
 */

TranslatorROSJAUS::TranslatorROSJAUS() {

}

/**
 * @brief Obtiene mensaje JAUS - Report Wrench Effort a partir de inforamcion del vehículo
 * @param subDest       Subsistema de destino
 * @param nodDest       Nodo de destino
 * @param steer         Valor de lectura de la dirección
 * @param throttle      Valor de lectura del acelerador
 * @param brake         Valor de lectura del freno de servicio
 * @return              Mensaje JAUS - Report Wrench Effort
 */
// Mensaje con informacion para Report Wrench Effort

JausMessage TranslatorROSJAUS::getJausMsgFromWrenchEffortInfo(int subDest, int nodDest, short steer, short throttle, short brake){
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_PRIMITIVE_DRIVER;
    jAdd->instance = 2;
    
    // Traduccion
        
    ReportWrenchEffortMessage rwem = reportWrenchEffortMessageCreate();
    rwem->presenceVector = (PRESENCE_VECTOR_THROTTLE | PRESENCE_VECTOR_BRAKE | PRESENCE_VECTOR_STEER);
    rwem->propulsiveLinearEffortXPercent = throttle;
    rwem->resistiveLinearEffortXPercent = brake;
    rwem->resistiveRotationalEffortZPercent = steer;
    jausAddressCopy(rwem->destination, jAdd);
    // Generacion de mensaje JUAS global
    jMsg = reportWrenchEffortMessageToJausMessage(rwem);
    reportWrenchEffortMessageDestroy(rwem);
    
    return jMsg;
    
}

// Mensaje de informacion para Report Discrete Device

JausMessage TranslatorROSJAUS::getJausMsgFromDiscreteDeviceInfo(int subDest, int nodDest, bool parkingbrake, short gear){
    
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_PRIMITIVE_DRIVER;
    jAdd->instance = 2;
    
    // Traduccion
    
    ReportDiscreteDevicesMessage rddm = reportDiscreteDevicesMessageCreate();
    rddm->presenceVector = (PRESENCE_VECTOR_PARKING_BRAKE | PRESENCE_VECTOR_GEAR);
    rddm->parkingBrake = (JausBoolean) parkingbrake;
    rddm->gear = gear;
    jausAddressCopy(rddm->destination, jAdd);
    // Generacion de mensaje JUAS global
    jMsg = reportDiscreteDevicesMessageToJausMessage(rddm);
    reportDiscreteDevicesMessageDestroy(rddm);
    
    return jMsg;
}

// Mensaje de informacion para Report Travel Speed
JausMessage TranslatorROSJAUS::getJausMsgFromTravelSpeedInfo(int subDest, int nodDest, short speed){
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_VELOCITY_STATE_SENSOR;
    jAdd->instance = 2;
    
    // Traduccion
    
    ReportTravelSpeedMessage rtsm = reportTravelSpeedMessageCreate();
    // Presence vector
    rtsm->speedMps = speed;
    jausAddressCopy(rtsm->destination, jAdd);
    // Generacion de mensaje JAUS global
    jMsg = reportTravelSpeedMessageToJausMessage(rtsm);
    reportTravelSpeedMessageDestroy(rtsm);
    
    return jMsg;
}

// Mensaje de informacion para UGV Info 

JausMessage TranslatorROSJAUS::getJausMsgFromUGVInfo(int subDest, int nodDest, short motorRPM, short motorTemperature){
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_PRIMITIVE_DRIVER;
    jAdd->instance = 2;
    
    // Traduccion
    
    UGVInfo12Message ugvm = ugvInfo12MessageCreate();
    // Presence vector 
    ugvm->presenceVector = (PRESENCE_VECTOR_MOTOR_RPM | PRESENCE_VECTOR_MOTOR_TEMPERATURE);
    ugvm->motor_rpm = motorRPM;
    ugvm->motor_temperature = motorTemperature;
    jausAddressCopy(ugvm->destination, jAdd);
    // Generacion de mensaje JAUS global
    jMsg = ugvInfo12MessageToJausMessage(ugvm);
    ugvInfo12MessageDestroy(ugvm);
    
    return jMsg;
}

// Mensaje de informacion para Report Signaling Elements

JausMessage TranslatorROSJAUS::getJausMsgFromSignalingInfo(int subDest, int nodDest, bool blinker_left, bool blinker_right, bool dipsp, bool dipss, bool dipsr, bool klaxon) {
     // Mensaje de devoluvion
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 2;
    
    // Traduccion
    
    ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
    // Presence vector
    rsem->blinker_right = (JausBoolean) blinker_right;
    rsem->blinker_left = (JausBoolean) blinker_left;
    rsem->dipsp = (JausBoolean) dipsp;
    rsem->dipss = (JausBoolean) dipss;
    rsem->dipsr = (JausBoolean) dipsr;
    rsem->klaxon = (JausBoolean) klaxon;
    jausAddressCopy(rsem->destination, jAdd);
    // Generacion de mensaje JAUS global
    jMsg = reportSignalingElements25MessageToJausMessage(rsem);
    reportSignalingElements25MessageDestroy(rsem);
    
    return jMsg;
}

// Mensajes de informacion electrica

JausMessage TranslatorROSJAUS::getJausMsgFromElectricInfo(int subDest, int nodDest, short bat_level, short bat_voltage, short bat_current, short bat_temp, short alarms) {

    // Mensaje de devoluvion
    JausMessage jMsg = NULL;
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_PRIMITIVE_DRIVER;
    jAdd->instance = 2;
    
    UGVInfo12Message ugvm = ugvInfo12MessageCreate();
    ugvm->presenceVector = 0x00 |
            PRESENCE_VECTOR_BATTERY_LEVEL |
            PRESENCE_VECTOR_BATTERY_VOLTAGE |
            PRESENCE_VECTOR_BATTERY_CURRENT |
            PRESENCE_VECTOR_BATTERY_TEMPERATURE |
            PRESENCE_VECTOR_ALARMS;
    ugvm->battery_level = bat_level;
    ugvm->battery_voltage = bat_voltage;
    ugvm->battery_current = bat_current;
    ugvm->battery_temperature = bat_temp;
    ugvm->alarms = alarms;
    jMsg = ugvInfo12MessageToJausMessage(ugvm);
    ugvInfo12MessageDestroy(ugvm);

    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    // Destruccion del mensaje
    return jMsg;
    
}

// Mensajes de informacion de camaras de apoyo a la conduccion

JausMessage TranslatorROSJAUS::getJausMsgFromCameraInfo(int subDest, int nodDest, short id_camera, short pan, short tilt, short zoom){
     
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 2;

    // Generacion de mensaje especifico UGV Info
    ReportCameraPoseMessage rcpm = reportCameraPoseMessageCreate();
    rcpm->cameraID = id_camera;
    rcpm->presenceVector = (PRESENCE_VECTOR_CURRENT_PAN | PRESENCE_VECTOR_CURRENT_TILT | PRESENCE_VECTOR_CURRENT_ZOOM);
    rcpm->xCameraAxisDirectionCosineZ = pan;
    rcpm->xCameraAxisDirectionCosineY = tilt;
    rcpm->zCameraOriginMeters = zoom;
    jausAddressCopy(rcpm->destination, jAdd);
    // Generacion de mensaje JUAS global
    jMsg = reportCameraPoseMessageToJausMessage(rcpm);
    reportCameraPoseMessageDestroy(rcpm);

    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    // Destruccion del mensaje
    return jMsg;
}

// Mensaje de informacion de camara IR

JausMessage TranslatorROSJAUS::getJausMsgFromIRCameraInfo(int subDest, int nodDest, short zoom, short polarity){
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;
    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 2;
    
    // Mensaje especifico
    ReportNightTimeCamera24Message ircm = reportNightTimeCamera24MessageCreate();
    ircm->active_zoom = zoom;
    if(polarity == 0){
        ircm->active_polarity = JAUS_FALSE;
    }else if(polarity == 1){
        ircm->active_polarity = JAUS_TRUE;
    }
    
    jausAddressCopy(ircm->destination, jAdd);
    
    // Generacion de mensaje JUAS global
    jMsg = reportNightTimeCamera24MessageToJausMessage(ircm);
    reportNightTimeCamera24MessageDestroy(ircm);
    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    return jMsg;
}

// Mensaje de informacion de telemetro

JausMessage TranslatorROSJAUS::getJausMsgFromTelemeterInfo(int subDest, int nodDest, short *ecs){
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;
    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_PLATFORM_SENSOR;
    jAdd->instance = 2;
    
    // Mensaje especifico
    ReportTelemeter27Message tim = reportTelemeter27MessageCreate();
    for(int i = 0; i < 5; i++){
        tim->echoes[i] = ecs[i];
    }
    
    jausAddressCopy(tim->destination, jAdd);
    
    // Generacion de mensaje JUAS global
    jMsg = reportTelemeter27MessageToJausMessage(tim);
    reportTelemeter27MessageDestroy(tim);
    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    return jMsg;
}

// Mensaje de informacion de camara TV

JausMessage TranslatorROSJAUS::getJausMsgFromTVCamera(int subDest, int nodDest, short zoom, short focus, bool autofocus){
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;
    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 2;
    
    // Mensaje especifico
    ReportDayTimeCamera22Message dtcm = reportDayTimeCamera22MessageCreate();
    dtcm->active_zoom = zoom;
    dtcm->active_focus = focus;
    dtcm->active_autofocus = (JausBoolean) autofocus;
    
    jausAddressCopy(dtcm->destination, jAdd);
    
    // Generacion de mensaje JUAS global
    jMsg = reportDayTimeCamera22MessageToJausMessage(dtcm);
    reportDayTimeCamera22MessageDestroy(dtcm);
    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    return jMsg;
}

// Mensaje de posicionador

JausMessage TranslatorROSJAUS::getJausMsgFromPositioner(int subDest, int nodDest, short pan, short tilt){
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;
    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_VISUAL_SENSOR;
    jAdd->instance = 2;
    
    // Mensaje especifico
    ReportPositioner20Message posm = reportPositioner20MessageCreate();
    posm->active_pan = pan;
    posm->active_tilt = tilt;
    
    jausAddressCopy(posm->destination, jAdd);
    
    // Generacion de mensaje JUAS global
    jMsg = reportPositioner20MessageToJausMessage(posm);
    reportPositioner20MessageDestroy(posm);
    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    return jMsg;
}
