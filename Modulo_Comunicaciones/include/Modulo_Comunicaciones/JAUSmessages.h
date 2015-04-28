/**
 * @file   JAUSmessages.h
 * @brief  Fichero de cabecera con los mensajes JAUS utilizados
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 * @addtogroup CommVehicle
 * @{
 */



#ifndef JAUSMESSAGES_H
#define	JAUSMESSAGES_H

#include <Modulo_Comunicaciones/constantCommunication.h>

/**
 * \struct mensajeJAUS
 * \brief  Estructura con todos los mensaje JAUS
 */
// Estructura de datos que maneja los mensajes JAUS
struct mensajeJAUS{

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

};

#endif	/* JAUSMESSAGES_H */

/**
 *@}
 */