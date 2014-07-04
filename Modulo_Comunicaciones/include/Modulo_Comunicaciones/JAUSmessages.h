/* 
 * File:   ConversionTypes.h
 * Author: atica
 *
 * Created on 28 de abril de 2014, 12:52
 */

#ifndef JAUSMESSAGES_H
#define	JAUSMESSAGES_H

#include <Modulo_Comunicaciones/constantCommunication.h>

// Estructura de datos que maneja los mensajes JAUS
typedef struct{

    ReportImageMessage image;
    ReportGlobalPoseMessage posGPS;
    ReportVelocityStateMessage velGPS;
    ReportErrorMessage error;

    SetWrenchEffortMessage   mainCommand;
    SetDiscreteDevicesMessage auxCommand;


    ReportGlobalWaypointMessage waypoint;
    ReportWaypointCountMessage numberWaypoints;
    RunMissionMessage startMode;
    PauseMissionMessage pauseMode;
    ResumeMissionMessage resumeMode;
    AbortMissionMessage exitMode;
    ReportMissionStatusMessage missionStatus;
    
    //BACKUP
    ReportWrenchEffortMessage backupWrench;
    ReportDiscreteDevicesMessage backupDiscrete;
    ReportVelocityStateMessage backupSpeed;
    
    //Ficheros
    ReportFileDataMessage file;
    QueryFileDataMessage  petFile;
    
    //Funciones auxiliares
    SetFunctionAuxiliarMessage faux;
    ReportFunctionAuxiliarMessage fauxACK;
    
    //Disponibilidad
    ReportAvailableMessage avail;
    
    //Información tipo de parada
    ReportInfoStopMessage infoStop;
    
    //Comandos de cámaras
    SetCameraPoseMessage camPose;
    SetCameraCapabilitiesMessage camZoom;

}mensajeJAUS;

#endif	/* JAUSMESSAGES_H */

