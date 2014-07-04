/* 
 * File:   gps.h
 * Author: carlosamores
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

#include <Common_files/msg_error.h>
#include <Common_files/msg_ctrl_camera.h>
#include "../../../Common_files/include/Common_files/constant.h"
#include <Modulo_Camaras/ProtPelcoD.h>


// ROS y demas librerias
#include "ros/ros.h"
#define NAME_SERIAL "/dev/ttyUSB0"
#define VELOCITY B9600

//Variables
bool exitModule;
ProtPelcoD*camara;
bool cameraAlive;


//Subscriptores
ros::Subscriber sub_cam;

// Publicadores
ros::Publisher pub_error;

// Funciones propias

bool connect();
bool disconnect();
bool configure();
bool sendData();
bool recvData();
bool isAlive();
bool checkStateCamera();

//Funciones de subscripcion
void fcn_sub_ctrl_camera(Common_files::msg_ctrl_camera msg);
