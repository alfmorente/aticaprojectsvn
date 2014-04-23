/* 
 * File:   gps.h
 * Author: atica
 *
 * Created on 16 de septiembre de 2013, 11:51
 */

#ifndef GPS_H
#define	GPS_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* GPS_H */

// Mensajes
#include "Common_files/msg_gps.h"
#include "Common_files/msg_error.h"
#include "Common_files/msg_module_enable.h"
#include "Common_files/msg_stream.h"
#include "Common_files/msg_backup.h"

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

// Señales externas
#include "external_signals.h"

// Interaccion con usuario
#include "interaction.h"

// Driver
#include "Modulo_GPS/driverNovatelFlexPakG2V1.h"


// Variable de continuacion de modulo
bool exitModule, readyToPublish, teachActive, launchTeach;
int stateOfGPS, numOfEmptyFrames;
// Variable donde guardar datos de INS
// Datos tramas
int hora, minutos, segundos, decsegundos, gpsquality, numsatellites;
float latitude, longitude;
char dirlatitude, dirlongitude, unit, checksum[2];
float precision, altitude;
float track_true, track_mag, knot_speed, km_speed;
char t_indicator, m_indicator, k_indicator, km_indicator;

// Estructura para almacenar los datos del gps
typedef struct{
    float latitude;
    float longitude;
    float altitude;
    float roll;
    float pitch;
    float yaw;
}insData;

// Variable donde guardar datos de INS
insData insdata;
// Hilo que controla el teach
TeachThread teachThread;
// Estructura para cola de datos en el hilo de teach
TeachData teachData;
// Frecuencia de muestre del GPS
int freq;

char serial[] = "/dev/ttyUSB0";

ros::Publisher pub_gps;
ros::Publisher pub_errores;
ros::Publisher pub_stream;
int res;

// Suscriptores
void fcn_sub_enableModule(const Common_files::msg_module_enable);
void fcn_sub_backup(const Common_files::msg_backup);

driverNovatelFlexPakG2V1 gps(&res, serial);


void getData(void); // Posicion

// Mensajes a publicar
Common_files::msg_gps insMessage;
Common_files::msg_error errMessage;




