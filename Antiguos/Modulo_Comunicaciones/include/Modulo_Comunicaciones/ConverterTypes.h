/* 
 * File:   ConversionTypes.h
 * Author: atica
 *
 * Created on 28 de abril de 2014, 12:52
 */

#ifndef CONVERSIONTYPES_H
#define	CONVERSIONTYPES_H

#include <string>
#include <sstream>


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

#include "../../libjaus/include/jaus.h"
#include "../../libopenJaus/include/openJaus.h"
#include "../../../Common_files/include/Common_files/constant.h"

#include <Modulo_Comunicaciones/JausSubsystemVehicle.h>
#include <Modulo_Comunicaciones/NodeROSCommunication.h>

using namespace std;

//Valores JAUS
#define JAUS_HIGH 25
#define JAUS_NEUTRAL_HIGH 76
#define JAUS_REVERSE 127
#define JAUS_NEUTRAL_LOW 178
#define JAUS_LOW 229

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

#define NO_ACK 0
#define ACK_MODE 1
#define ACK_ERROR 2
#define ACK_AVAILABLE 3
#define ACK_FUNC_AUX 3

#define TIMEOUT_ACK 5

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
    ReportFunctionAuxiliarMessage fauxACK;
    
    //Disponibilidad
    ReportAvailableMessage avail;
    
    //Información tipo de parada
    ReportInfoStopMessage infoStop;
    
    //Comandos de cámaras
    SetCameraPoseMessage camPose;
    SetCameraCapabilitiesMessage camZoom;

}mensajeJAUS;

JausMessage convertROStoJAUS(ROSmessage msg_ROS);
ROSmessage convertJAUStoROS(JausMessage msg_JAUS);
int redondea(float valor);

#endif	/* CONVERSIONTYPES_H */

