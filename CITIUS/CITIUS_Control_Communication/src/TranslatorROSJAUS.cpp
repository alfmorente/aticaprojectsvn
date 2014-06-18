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
            rddm->parkingBrake = (JausBoolean)value;
            jausAddressCopy(rddm->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = reportDiscreteDevicesMessageToJausMessage(rddm);
            reportDiscreteDevicesMessageDestroy(rddm);
            break;
        }case STEERING:{
            // Generacion de mensaje especifico Report Wrench Effort
            ReportWrenchEffortMessage rwem = reportWrenchEffortMessageCreate();
            rwem->resistiveRotationalEffortZPercent = value;
            jausAddressCopy(rwem->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = reportWrenchEffortMessageToJausMessage(rwem);
            reportWrenchEffortMessageDestroy(rwem);
            break;
        }case GEAR:{
            // Generacion de mensaje especifico Report Discrete Device
            ReportDiscreteDevicesMessage rddm = reportDiscreteDevicesMessageCreate();
            rddm->gear = value;
            jausAddressCopy(rddm->destination,jAdd);
            // Generacion de mensaje JUAS global
            jMsg = reportDiscreteDevicesMessageToJausMessage(rddm);
            reportDiscreteDevicesMessageDestroy(rddm);
            break;
        }case MOTOR_RPM:{
            // Generacion de mensaje especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            ugvm->rpmMotor = value;
            jausAddressCopy(ugvm->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
            break;
        }case MOTOR_TEMPERATURE:{
            // Generacion de mensaje especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            ugvm->temperaturaMotor = value;
            jausAddressCopy(ugvm->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
            break;
        }case CRUISING_SPEED:{
            // Generacion de mensaje especifico Report velocity state
            ReportVelocityStateMessage rvsm = reportVelocityStateMessageCreate();
            rvsm->velocityRmsMps = value;
            jausAddressCopy(rvsm->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportVelocityStateMessageToJausMessage(rvsm);
            reportVelocityStateMessageDestroy(rvsm);
            break;
        }case BLINKER_RIGHT:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            rsem->intermitenteDerechoActivo = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case BLINKER_LEFT:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            rsem->intermitenteIzquierdoActivo = (JausBoolean) value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case BLINKER_EMERGENCY:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            rsem->intermitenteDerechoActivo = (JausBoolean)value;
            rsem->intermitenteIzquierdoActivo = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case DIPSP:{
            // TODO
            
            break;
        }case DIPSS:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            rsem->lucesCortasActivas = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case DIPSR:{
            // TODO
            
            break;
        }case KLAXON:{
            // Generacion de mensaje especifico Report signlaling elements
            ReportSignalingElements25Message rsem = reportSignalingElements25MessageCreate();
            rsem->claxonActivo = (JausBoolean)value;
            jausAddressCopy(rsem->destination,jAdd);
            // Generacion de mensaje JAUS global
            jMsg = reportSignalingElements25MessageToJausMessage(rsem);
            reportSignalingElements25MessageDestroy(rsem);
            break;
        }case DRIVE_ALARMS:{
            // Generacion de mensaje especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            ugvm->alarmas = value;
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
            ugvm->nivelDeBateria = value;
            jausAddressCopy(ugvm->destination, jAdd);
            // Generacion de mensaje JUAS global
            jMsg = ugvInfo12MessageToJausMessage(ugvm);
            ugvInfo12MessageDestroy(ugvm);
            break;
        }case BATTERY_VOLTAGE:{
            // Generacion de mensaje especifico UGV Info
            UGVInfo12Message ugvm = ugvInfo12MessageCreate();
            ugvm->tensionBateria = value;
            jausAddressCopy(ugvm->destination, jAdd);
            // Generacion de mensaje JUAS global
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

// Mensajes de informacion de camara

JausMessage TranslatorROSJAUS::getJausMsgFromCameraInfo(int subDest, int nodDest, short id_camera, short pan, short tilt){
     
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
    rcpm->xCameraAxisDirectionCosineZ = pan;
    rcpm->xCameraAxisDirectionCosineY = tilt;
    jausAddressCopy(rcpm->destination, jAdd);
    // Generacion de mensaje JUAS global
    jMsg = reportCameraPoseMessageToJausMessage(rcpm);
    reportCameraPoseMessageDestroy(rcpm);

    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    // Destruccion del mensaje
    return jMsg;
}

// Mensajes de informacion de posicion / orientacion

JausMessage TranslatorROSJAUS::getJausMsgFromPosOriInfo(int subDest, int nodDest, double lat, double lon, double alt, double roll, double yaw, double pitch){
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_GLOBAL_POSE_SENSOR;

    // Generacion de mensaje especifico UGV Info
    ReportGlobalPoseMessage rgpm = reportGlobalPoseMessageCreate();
    rgpm->latitudeDegrees = lat;
    rgpm->longitudeDegrees = lon;
    rgpm->attitudeRmsRadians = alt;
    rgpm->rollRadians = roll;
    rgpm->pitchRadians = pitch;
    rgpm->yawRadians = yaw;
    jausAddressCopy(rgpm->destination, jAdd);
    // Generacion de mensaje JUAS global
    jMsg = reportGlobalPoseMessageToJausMessage(rgpm);
    reportGlobalPoseMessageDestroy(rgpm);

    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    // Destruccion del mensaje
    return jMsg;
}

// Mensaje de informacion de modo de operacion

JausMessage TranslatorROSJAUS::getJausMsgFromStatus(int subDest, int nodDest, int status){
    // Mensaje de devoluvion
    JausMessage jMsg = NULL;
    // Creacion de la direccion destinataria
    JausAddress jAdd = jausAddressCreate();
    jAdd->subsystem = subDest;
    jAdd->node = nodDest;
    jAdd->component = JAUS_MISSION_SPOOLER;

    // Generacion de mensaje especifico UGV Info
    ReportMissionStatusMessage rmsm = reportMissionStatusMessageCreate();
    rmsm->missionId = status;
    jausAddressCopy(rmsm->destination, jAdd);
    // Generacion de mensaje JUAS global
    jMsg = reportMissionStatusMessageToJausMessage(rmsm);
    reportMissionStatusMessageDestroy(rmsm);

    // Destruccion de la estructura destinatario
    jausAddressDestroy(jAdd);
    // Destruccion del mensaje
    return jMsg;
}
