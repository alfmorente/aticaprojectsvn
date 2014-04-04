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
#include "Common_files/msg_camera.h"
#include "Common_files/msg_ctrl_camera.h"
#include "Common_files/msg_com_teleop.h"
#include "Common_files/msg_error.h"
#include "Common_files/msg_gps.h"
#include "Common_files/msg_mode.h"
#include "Common_files/msg_waypoint.h"
#include "Common_files/msg_backup.h"
#include "Common_files/msg_available.h"
#include "Common_files/msg_fcn_aux.h"
#include "Common_files/msg_stream.h"
#include "Common_files/msg_info_stop.h"
#include "Common_files/srv_data.h"

//ROS y demás librerias
#include "ros/ros.h"
#include "../../../Common_files/include/Common_files/constant.h"
#include "../../libjaus/include/jaus.h"
#include "../../libopenJaus/include/openJaus.h"
#include "Modulo_Comunicaciones/handlerJAUS.h"

//Constantes propias del módulo
// Tipo de mensaje ROS (para traducir a JAUS y viceversa)
#define TOM_CAMERA 0
#define TOM_GPS 1
#define TOM_ERROR 2
#define TOM_MODE 3
#define TOM_WAYPOINT 4
#define TOM_REMOTE 6
#define TOM_FUNC_AUX 7
#define TOM_BACKUP_WRENCH 8
#define TOM_BACKUP_DISCRETE 9
#define TOM_BACKUP_SPEED 10
#define TOM_PET_FILE 11
#define TOM_FILE 12
#define TOM_AVAILABLE 13
#define TOM_INFO_STOP 14
#define TOM_CTRL_CAMERA 15
#define TOM_UNKNOW -1

//Valores JAUS
#define JAUS_HIGH 25
#define JAUS_NEUTRAL_HIGH 76
#define JAUS_REVERSE 127
#define JAUS_NEUTRAL_LOW 178
#define JAUS_LOW 229

#define MISSION 0
#define TASK 1

#define SPOOLING 0
#define PENDINGM 1
#define PAUSED 2
#define ABORTED 3
#define FINISHED 4

//Estado de las comunicaciones
#define COM_OFF 0
#define COM_ON 1
#define COM_LOSED 2

//Presences VectorS
#define PRESENCE_VECTOR_THROTTLE 0X0001
#define PRESENCE_VECTOR_STEER 0X0010
#define PRESENCE_VECTOR_BRAKE 0X0040

#define PRESENCE_VECTOR_ENGINE 0X01
#define PRESENCE_VECTOR_PARKING_BRAKE 0X02
#define PRESENCE_VECTOR_LIGHT_IR 0X22
#define PRESENCE_VECTOR_LIGHT_CONVENTIONAL 0X32
#define PRESENCE_VECTOR_DIFERENTIAL_LOCK 0X42
#define PRESENCE_VECTOR_ENABLE_LASER2D 0X52
#define PRESENCE_VECTOR_GEAR 0X04

#define PRESENCE_VECTOR_PAN 0X01
#define PRESENCE_VECTOR_TILT 0X02
#define PRESENCE_VECTOR_HOME 0X03
#define PRESENCE_VECTOR_ZOOM 0X04











//Clientes
ros::ServiceClient clientMode;

// Publicadores
// comentario
ros::Publisher pub_mode;
ros::Publisher pub_comteleop_unclean;
ros::Publisher pub_comteleop_clean;
ros::Publisher pub_waypoint;
ros::Publisher pub_error;
ros::Publisher pub_fcn_aux;
ros::Publisher pub_plan;
ros::Publisher pub_ctrl_camera;


//Variables JAUS del subsystema
OjCmpt compCamera;
OjCmpt compGPS;
OjCmpt compVehicle;
OjCmpt compMission;
OjCmpt compNavigation;
OjCmpt compVelSensor;


MyHandler* handler;
FileLoader *configData;
NodeManager* nm;
int communicationState;

// Otras Variables
int numberWaypoints; //Numero de waypoints del camino

// Estructura de datos que maneja los mensajes ROS
typedef struct{
    short tipo_mensaje;
    Common_files::msg_camera mens_cam;
    Common_files::msg_gps mens_gps;
    Common_files::msg_error mens_error;
    Common_files::msg_mode mens_mode;
    Common_files::msg_waypoint mens_waypoint;
    Common_files::msg_com_teleop mens_teleop;
    Common_files::msg_backup mens_backup;
    Common_files::msg_available mens_available;
    Common_files::msg_stream mens_file; 
    Common_files::msg_fcn_aux mens_fcn_aux; 
    Common_files::msg_info_stop mens_info_stop;
    Common_files::msg_ctrl_camera mens_ctrl_cam;
    
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
    
    //BACKUP
    ReportWrenchEffortMessage backupWrench;
    ReportDiscreteDevicesMessage backupDiscrete;
    ReportVelocityStateMessage backupSpeed;
    
    //Ficheros
    ReportFileDataMessage file;
    QueryFileDataMessage  petFile;
    
    //Funciones auxiliares
    SetFunctionAuxiliarMessage faux;
    
    //Disponibilidad
    ReportAvailableMessage avail;
    
    //Información tipo de parada
    ReportInfoStopMessage infoStop;
    
    //Comandos de cámaras
    SetCameraPoseMessage camPose;
    SetCameraCapabilitiesMessage camZoom;

}mensajeJAUS;


//Funcion para el spin de ROS
void* spinThread(void* obj);

// Funciones de suscripcion
void fcn_sub_gps(const Common_files::msg_gps);
void fcn_sub_error(const Common_files::msg_error);
void fcn_sub_camera(const Common_files::msg_camera);
void fcn_sub_mode(const Common_files::msg_mode);
void fcn_sub_backup(const Common_files::msg_backup);
void fcn_sub_available(const Common_files::msg_available);
void fcn_sub_teach_file(const Common_files::msg_stream);
void fcn_sub_info_stop(const Common_files::msg_info_stop);


// Funciones propias
bool connect();
void disconnect();
bool checkConnection();
bool configureJAUS();
JausMessage convertROStoJAUS(ROSmessage msg_ROS);
ROSmessage convertJAUStoROS(mensajeJAUS msg_JAUS);
void sendJAUSMessage(JausMessage txMessage);
void rcvJAUSMessage(OjCmpt comp,JausMessage rxMessage);
int redondea(float valor);


//Funciones auxiliares
string getDebugConfiguration();
bool setDebugConfiguration(string);
void losedCommunication();
int getStateModule(ros::NodeHandle);
void setStateModule(ros::NodeHandle,int);

