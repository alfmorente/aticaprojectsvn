
/** 
 * @file  gps.h
 * @brief Fichero de cabecera para funciones de gestióin del GPS+IMU
 * @author Carlos Amores
 * @date 2013, 2014, 2015
 */
#ifndef GPS_H
#define	GPS_H

#endif	/* GPS_H */

// Mensajes
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_gps.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_error.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_module_enable.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_stream.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_backup.h"
// Librerias 
#include <iostream>
#include "TeachThread.hpp"
#include <time.h>
#include <stdlib.h> 
#include <stdio.h>
// ROS
#include "ros/ros.h"
#include "../../../Common_files/include/Common_files/constant.h"
#include "Modulo_GPS/constant_gps.h"
#include "Modulo_GPS/GPS_Management.h"
// Señales externas
#include "external_signals.h"
// Interaccion con usuario
#include "interaction.h"

/**
 * \struct insData
 * \brief Estructura para almacenamiento de información recibida del GPS+IMU
 */
typedef struct{
    float latitude;
    float longitude;
    float altitude;
    float roll;
    float pitch;
    float yaw;
}insData;


bool exitModule, readyToPublish, teachActive;
insData insdata;
TeachThread teachThread;
TeachData teachData;
void fcn_sub_enableModule(const Common_files::msg_module_enable);
void fcn_sub_backup(const Common_files::msg_backup);
bool connect();
bool disconnect();
bool configure();
bool sendData();
bool recvData();
bool isAlive();
bool checkStateGPS();
void initModuleVariables();



