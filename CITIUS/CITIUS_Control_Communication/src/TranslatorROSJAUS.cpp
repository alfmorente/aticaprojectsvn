#include "TranslatorROSJAUS.h"
#include "constant.h"
#include "message/experimental/ugvInfo12Message.h"
#include "message/experimental/reportSignalingElements25Message.h"

TranslatorROSJAUS::TranslatorROSJAUS() {

}

// Mensajes de informacion de vehiculos

JausMessage TranslatorROSJAUS::getJausMsgFromVehicleInfo(int subDest, int nodDest, short id_device, short value){
    
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;    
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_PRIMITIVE_DRIVER;
    

    // Traduccion
    switch (id_device) {
        case THROTTLE:{
            // Generacion de mensaje especifico Report Wrench Effort
            ReportWrenchEffortMessage rwem = reportWrenchEffortMessageCreate();
            // Presence vector
            rwem->presenceVector = PRESENCE_VECTOR_THROTTLE;
            rwem->propulsiveLinearEffortXPercent = value;
            jausAddressCopy(rwem->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = reportWrenchEffortMessageToJausMessage(rwem);
            reportWrenchEffortMessageDestroy(rwem);
            break;
        }case BRAKE:{
            // Generacion de mensaje especifico Report Wrench Effort
            ReportWrenchEffortMessage rwem = reportWrenchEffortMessageCreate();
            // Presence vector
            rwem->presenceVector = PRESENCE_VECTOR_BRAKE;
            rwem->resistiveLinearEffortXPercent = value;
            jausAddressCopy(rwem->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = reportWrenchEffortMessageToJausMessage(rwem);
            reportWrenchEffortMessageDestroy(rwem);
            break;
        }case HANDBRAKE:{
            // Generacion de mensaje especifico Report Discrete Device
            ReportDiscreteDevicesMessage rddm = reportDiscreteDevicesMessageCreate();
            // Presence vector 
            rddm->presenceVector = PRESENCE_VECTOR_PARKING_BRAKE;
            rddm->parkingBrake = (JausBoolean)value;
            jausAddressCopy(rddm->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = reportDiscreteDevicesMessageToJausMessage(rddm);
            reportDiscreteDevicesMessageDestroy(rddm);
            break;
        }case STEERING:{
            // Generacion de mensaje especifico Report Wrench Effort
            ReportWrenchEffortMessage rwem = reportWrenchEffortMessageCreate();
            // Presence vector
            rwem->presenceVector = PRESENCE_VECTOR_STEER;
            rwem->resistiveRotationalEffortZPercent = value;
            jausAddressCopy(rwem->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = reportWrenchEffortMessageToJausMessage(rwem);
            reportWrenchEffortMessageDestroy(rwem);
            break;
        }case GEAR:{
            // Generacion de mensaje especifico Report Discrete Device
            ReportDiscreteDevicesMessage rddm = reportDiscreteDevicesMessageCreate();
            // Presence vector
            rddm->presenceVector = PRESENCE_VECTOR_GEAR;
            rddm->gear = value;
            jausAddressCopy(rddm->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = reportDiscreteDevicesMessageToJausMessage(rddm);
            reportDiscreteDevicesMessageDestroy(rddm);
            break;
        }case MOTOR_RPM:{
            // Generacion de mensaje especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            // Presence vector 
            ugvm->presenceVector = PRESENCE_VECTOR_MOTOR_RPM;
            ugvm->motor_rpm = value;
            jausAddressCopy(ugvm->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
            break;
        }case MOTOR_TEMPERATURE:{
            // Generacion de mensaje especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            // Presence vector
            ugvm->presenceVector = PRESENCE_VECTOR_MOTOR_TEMPERATURE;
            ugvm->motor_temperature = value;
            jausAddressCopy(ugvm->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
            break;
        }case CRUISING_SPEED:{
            // Generacion de mensaje especifico Report travel speed state
            ReportTravelSpeedMessage rvsm = reportTravelSpeedMessageCreate();
            // Presence vector
            rvsm->speedMps = value;
            jausAddressCopy(rvsm->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportTravelSpeedMessageToJausMessage(rvsm);
            reportTravelSpeedMessageDestroy(rvsm);
            break;
        }case BLINKER_RIGHT:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            // Presence vector
            rsem->presenceVector = PRESENCE_VECTOR_BLINKER_RIGHT;
            rsem->blinker_right = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case BLINKER_LEFT:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            // Presence vector
            rsem->presenceVector = PRESENCE_VECTOR_BLINKER_LEFT;
            rsem->blinker_left = (JausBoolean) value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case BLINKER_EMERGENCY:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            // Presence vector
            rsem->presenceVector = (PRESENCE_VECTOR_BLINKER_RIGHT | PRESENCE_VECTOR_BLINKER_LEFT);
            rsem->blinker_right = (JausBoolean)value;
            rsem->blinker_left = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case DIPSP:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            // Presence vector
            rsem->presenceVector = PRESENCE_VECTOR_DIPSP;
            rsem->dipsp = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case DIPSS:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            // Presence vector
            rsem->presenceVector = PRESENCE_VECTOR_DIPSS;
            rsem->dipss = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case DIPSR:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            // Presence vector
            rsem->presenceVector = PRESENCE_VECTOR_DIPSR;
            rsem->dipsr = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case KLAXON:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            // Presence vector
            rsem->presenceVector = PRESENCE_VECTOR_KLAXON;
            rsem->klaxon = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case DRIVE_ALARMS:{
            // Generacion de mensaje especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            // Presence vector
            ugvm->presenceVector = PRESENCE_VECTOR_ALARMS;
            ugvm->alarms = value;
            jausAddressCopy(ugvm->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
            break;
        }default:{
            break;
        }
    };
        
    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    // Destruccion del mensaje
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
