/**
 * @file   laser2D_frontal.h
 * @brief  Fichero Cabecera de gestion del Modulo del Laser frontal
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 * @addtogroup FrontLaser
 * @{
 */

#ifndef LASER2D_H
#define	LASER2D_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* LASER2D_H */

// Mensajes
#include <Modulo_Laser2D_Frontal/Files.h>
#include <Modulo_Laser2D_Frontal/SickLMS5xx.h>
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_error.h"
#include "sensor_msgs/LaserScan.h"
#include "../../../Common_files/include/Common_files/constant.h"

// ROS
#include "ros/ros.h"

// NO hay suscriptores

//Publicadores
ros::Publisher pub_data;
ros::Publisher pub_error;

//Estructuras de datos del laser
queryInformation q; //nuevo
measuringParameters configParameters;
dataOutput configOutput;
conectionParameters configConection;
laserScan scanData;

//Objetos de clases
Files file; //Clase para operar con ficheros
Sicklms5xx miLaser;    //Clase para operar con el laser Sick LMS 111


//Mensajes
sensor_msgs::LaserScan rosLaser;

// Funciones propias del Módulo
int connect();
void disconnect();
int configure();
bool sendData();
int  recvData(laserScan* scanData,int timeout);
bool isAlive(int error);


//Metodos relacionados con ROS
void publicErrorToROS(int error);
void publicDataToROS(laserScan scandata);
void setStateModule(ros::NodeHandle n,int state);
int getStateModule(ros::NodeHandle n);

/**
 *@}
 */