/* 
 * File:   constantCommunication.h
 * Author: atica
 *
 * Created on 28 de abril de 2014, 12:52
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

#define PRESENCE_VECTOR_PAN 0X20
#define PRESENCE_VECTOR_TILT 0X10
#define PRESENCE_VECTOR_HOME 0X30
#define PRESENCE_VECTOR_ZOOM 0X0001

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

#endif //CONSTANT_COMMUNICATION.H
