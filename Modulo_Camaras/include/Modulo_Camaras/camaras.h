/**
 * @file   camaras.h
 * @brief  Fichero de cabecera de gestion de la camara
 * @author David Jimenez 
 * @date   2013, 2014, 2015
 */

#ifndef CAMARAS_H
#define	CAMARAS_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* GPS_H */

#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_error.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_ctrl_camera.h"
#include "../../../Common_files/include/Common_files/constant.h"
#include <Modulo_Camaras/ProtPelcoD.h>


// ROS y demas librerias
#include "ros/ros.h"
#define NAME_SERIAL "/dev/ttyUSB0" ///< Constante que define el nombre del puerto serie
#define VELOCITY B9600 ///< Constante que define la velocidad del puerto serie

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
