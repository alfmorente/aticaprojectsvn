/**
 * @file   constantCommunication.h
 * @brief  Fichero de constantes propias del módulo de comunicaciones
 * @author David Jiménez 
 * @date   2013, 2014, 2015
 * @addtogroup CommVehicle
 * @{
 */


#include <string>
#include <sstream>
#include "../../libjaus/include/jaus.h"
#include "../../libopenJaus/include/openJaus.h"
#include <string>
#include <sstream>

// Mensajes
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_camera.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_ctrl_camera.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_available.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_backup.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_com_teleop.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_emergency_stop.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_error.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_fcn_aux.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_gps.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_info_stop.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_mode.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_waypoint.h>
#include <../../Common_files/msg_gen/cpp/include/Common_files/msg_stream.h>
#include <../../Common_files/srv_gen/cpp/include/Common_files/srv_data.h>

using namespace std;

#ifndef CONSTANT_COMUNICATION_H
#define	CONSTANT_COMUNICATION_H


//Valores JAUS
#define JAUS_HIGH 25 ///< Constante para marcha HIGH
#define JAUS_NEUTRAL_HIGH 76 ///< Constante para marcha NEUTRAL HIGH 
#define JAUS_REVERSE 127 ///< Constante para marcha REVERSE
#define JAUS_NEUTRAL_LOW 178 ///< Constante para marcha NEUTRAL LOW
#define JAUS_LOW 229 ///< Constante para marcha LOW

//Estado de las comunicaciones
#define COM_OFF 0 ///< Constante para comunicación OFF
#define COM_ON 1 ///< Constante para comunicación ON
#define COM_LOSED 2 ///< Constante para comunicación perdida

//Presences VectorS
#define PRESENCE_VECTOR_THROTTLE 0X0001 ///< Constante para indicar comando Acelerador
#define PRESENCE_VECTOR_STEER 0X0010 ///< Constante para indicar comando de Giro
#define PRESENCE_VECTOR_BRAKE 0X0040 ///< Constante para indicar comando de freno

#define PRESENCE_VECTOR_ENGINE 0X01 ///< Constante para indicar comando de encendido del motor
#define PRESENCE_VECTOR_PARKING_BRAKE 0X02 ///< Constante para indicar comando de freno de mano
#define PRESENCE_VECTOR_LIGHT_IR 0X22 ///< Constante para indicar comando de luz infraroja
#define PRESENCE_VECTOR_LIGHT_CONVENTIONAL 0X32 ///< Constante para indicar luz convencional
#define PRESENCE_VECTOR_DIFERENTIAL_LOCK 0X42 ///<Constante para indicar bloqueo diferencial
#define PRESENCE_VECTOR_ENABLE_LASER2D 0X52 ///< Constante para indicar comando de hablitación/deshabilitación de laser 2D
#define PRESENCE_VECTOR_GEAR 0X04 ///< Constante para indicar comando de marcha

#define PRESENCE_VECTOR_PAN 0X20 ///< Constante para indicar comando de PAN a cámara
#define PRESENCE_VECTOR_TILT 0X10 ///< Constante para indicar comando de TILT a cámara
#define PRESENCE_VECTOR_HOME 0X30 ///< Constante para indicar comando de HOME a cámara
#define PRESENCE_VECTOR_ZOOM 0X0001 ///< Constante para indicar comando de ZOOM a cámara

#define NO_ACK 0 ///< Constante para indicar mensaje sin ack
#define ACK_MODE 1 ///< Constante para indicar ack de mensaje del modo
#define ACK_ERROR 2 ///< Constante para indicar ack de mensaje de error
#define ACK_AVAILABLE 3 ///< Constante para indicar ack del mensaje de disponibilidad de modos
#define ACK_FUNC_AUX 4 ///< Constante para indicar ack del mensaje de función auxiliar

#define TIMEOUT_ACK 5 ///< Constante para indicar timeout de espera de ack

// Tipo de mensaje ROS (para traducir a JAUS y viceversa)
//#define TOM_CAMERA 0 ///< Constante para indicar tipo de mensaje ROS de cámara
#define TOM_GPS 0 ///< Constante para indicar tipo de mensaje ROS de GPS
#define TOM_ERROR 1 ///< Constante para indicar tipo de mensaje ROS de error
#define TOM_MODE 2 ///< Constante para indicar tipo de mensaje ROS de modo
#define TOM_WAYPOINT 3 ///< Constante para indicar tipo de mensaje ROS de waypoint
#define TOM_REMOTE 4 ///< Constante para indicar tipo de mensaje ROS de comando de vehículo
#define TOM_FUNC_AUX 5 ///< Constante para indicar tipo de mensaje ROS de función auxiliar
#define TOM_BACKUP_WRENCH 6 ///< Constante para indicar tipo de mensaje ROS de backup principal
#define TOM_BACKUP_DISCRETE 7 ///< Constante para indicar tipo de mensaje ROS de backup auxiliar
#define TOM_BACKUP_SPEED 8 ///< Constante para indicar tipo de mensaje ROS de backup de velocidad
#define TOM_PET_FILE 9 ///< Constante para indicar tipo de mensaje ROS de peticion de fichero
#define TOM_FILE 10 ///< Constante para indicar tipo de mensaje ROS de envío de fichero
#define TOM_AVAILABLE 11 ///< Constante para indicar tipo de mensaje ROS de disponibilidad de los modos
#define TOM_INFO_STOP 12 ///< Constante para indicar tipo de mensaje ROS de estado de la parada de emergencia
#define TOM_CTRL_CAMERA 13 ///< Constante para indicar tipo de mensaje ROS para control de la cámara
#define TOM_UNKNOW -1 ///< Constante para indicar tipo de mensaje ROS desconocido

#endif //CONSTANT_COMMUNICATION.H

/**
 *@}
 */