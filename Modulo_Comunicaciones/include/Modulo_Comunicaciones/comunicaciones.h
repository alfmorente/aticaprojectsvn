/* 
 * File:   comunicaciones.h
 * Author: carlosamores
 *
 * Created on 12 de septiembre de 2013, 12:48
 */

#ifndef COMUNICACIONES_H
#define	COMUNICACIONES_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* COMUNICACIONES_H */

// Mensajes
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_camera.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_com_teleoperate.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_error.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_gps.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_mode.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_waypoints.h"
#include "../../msg_gen/cpp/include/Modulo_Comunicaciones/msg_backup.h"

//ROS y demás librerias
#include "ros/ros.h"
#include "constant.h"
#include "../../libjaus/include/jaus.h"
#include "../../libopenJaus/include/openJaus.h"
#include "../include/Modulo_Comunicaciones/handlerJAUS.h"


// Publicadores
// comentario
ros::Publisher pub_mode;
ros::Publisher pub_comteleop;
ros::Publisher pub_waypoints;
ros::Publisher pub_error;

//Variables JAUS del subsystema
OjCmpt compCamera;
OjCmpt compGPS;
OjCmpt compVehicle;
OjCmpt compMission;

MyHandler* handler;
FileLoader *configData;
NodeManager* nm;
bool comActiva;

// Otras Variables
int numberWaypoints; //Numero de waypoints del camino

// Estructura de datos que maneja los mensajes ROS
typedef struct{
    short tipo_mensaje;
    Modulo_Comunicaciones::msg_camera mens_cam;
    Modulo_Comunicaciones::msg_gps mens_gps;
    Modulo_Comunicaciones::msg_error mens_error;
    Modulo_Comunicaciones::msg_mode mens_mode;
    Modulo_Comunicaciones::msg_waypoints mens_waypoints;
    Modulo_Comunicaciones::msg_com_teleoperate mens_teleop;
    Modulo_Comunicaciones::msg_backup mens_backup;
}ROSmessage;

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

}mensajeJAUS;
short JAUS_message_type;

double* auxWaypointLat;
double* auxWaypointLon;

//Funcion para el spin de ROS
void* spinThread(void* obj);

// Funciones de suscripcion
void fcn_sub_gps(const Modulo_Comunicaciones::msg_gps);
void fcn_sub_error(const Modulo_Comunicaciones::msg_error);
void fcn_sub_camera(const Modulo_Comunicaciones::msg_camera);
void fcn_sub_mode(const Modulo_Comunicaciones::msg_mode);
void fcn_sub_backup(const Modulo_Comunicaciones::msg_backup);

//Funciones de Publicacion
void fcn_pub_mode(Modulo_Comunicaciones::msg_mode);
void fcn_pub_comteleop(Modulo_Comunicaciones::msg_com_teleoperate);
void fcn_pub_waypoints(Modulo_Comunicaciones::msg_waypoints);
void fcn_pub_error(Modulo_Comunicaciones::msg_error);

// Funciones propias
void connect();
void disconnect();
bool checkConnection();
bool configureJAUS();
JausMessage convertROStoJAUS(ROSmessage msg_ROS);
ROSmessage convertJAUStoROS(mensajeJAUS msg_JAUS);
void sendJAUSMessage(JausMessage txMessage);
void rcvJAUSMessage(OjCmpt comp,JausMessage rxMessage);
int redondea(float valor);
