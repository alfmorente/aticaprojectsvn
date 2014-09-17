#include "TranslatorROSJAUS.h"
#include "constant.h"
#include "message/experimental/ugvInfo12Message.h"
#include "message/experimental/reportSignalingElements25Message.h"

TranslatorROSJAUS::TranslatorROSJAUS() {

}

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

JausMessage TranslatorROSJAUS::getJausMsgFromElectricInfo(int subDest, int nodDest, short id_device, short value) {

    // Mensaje de devoluvion
    JausMessage jMsg = NULL;
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_PRIMITIVE_DRIVER;
    jAdd->instance = 2;

    // Traduccion
    switch (id_device) {
        case BATTERY_LEVEL:{
            // Generacion de mensaje especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            // Presence vector
            ugvm->presenceVector = PRESENCE_VECTOR_BATTERY_LEVEL;
            ugvm->battery_level = value;
            jausAddressCopy(ugvm->destination, jAdd);
            // Generacion de mensaje JUAS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
            break;
        }case BATTERY_VOLTAGE:{
            // Generacion de mensaje especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            // Presence vector
            ugvm->presenceVector = PRESENCE_VECTOR_BATTERY_VOLTAGE;
            ugvm->battery_voltage = value;
            jausAddressCopy(ugvm->destination, jAdd);
            // Generacion de mensaje JUAS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
            break;
        }case BATTERY_CURRENT:{
            // Generacion de mensjae especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            // Presence vector
            ugvm->presenceVector = PRESENCE_VECTOR_BATTERY_CURRENT;
            ugvm->battery_current = value;
            jausAddressCopy(ugvm->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
        }case BATTERY_TEMPERATURE:{
            // Generacion de mensjae especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            // Presence vector
            ugvm->presenceVector = PRESENCE_VECTOR_BATTERY_TEMPERATURE;
            ugvm->battery_temperature = value;
            jausAddressCopy(ugvm->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
        }case SUPPLY_ALARMS:{
            // Diferenciar entre estas alarmas y las del vehiculo
            // TODO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            
            // Generacion de mensjae especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            // Presence vector
            ugvm->presenceVector = PRESENCE_VECTOR_ALARMS;
            ugvm->alarms = value;
            jausAddressCopy(ugvm->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
        }default:{
            break;
        }
    };

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
    TelemeterInfo10Message tim = telemeterInfo10MessageCreate();
    for(int i = 0; i < 5; i++){
        tim->echoes[i] = ecs[i];
    }
    
    jausAddressCopy(tim->destination, jAdd);
    
    // Generacion de mensaje JUAS global
    jMsg = telemeterInfo10MessageToJausMessage(tim);
    telemeterInfo10MessageDestroy(tim);
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
