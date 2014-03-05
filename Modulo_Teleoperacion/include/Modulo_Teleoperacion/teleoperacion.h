/* 
 * File:   teleoperacion.h
 * Author: atica
 *
 * Created on 13 de septiembre de 2013, 11:27
 */

#ifndef TELEOPERACION_H
#define	TELEOPERACION_H

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* TELEOPERACION_H */

//Mensajes
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_com_teleop.h"
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_error.h"
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_module_enable.h"
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_laser.h"
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_mode.h"

//ROS y demás librerias
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "constant.h"

//Variables globales
// Manejador ROS
ros::NodeHandle n;
bool enableModule;
int error_count;
int LasFront_warning;
int LasFront_alarm;

//Funciones de suscripcion
void fcn_sub_com_teleop(const Modulo_Teleoperacion::msg_com_teleop);
void fcn_sub_enable_module(const Modulo_Teleoperacion::msg_module_enable);
void fcn_sub_laser(const Modulo_Teleoperacion::msg_laser);

//Funciones propias
int convertToCorrectValues(int,int);
// Devuelve true si el proccesado de datos dictamina peligro y false en caso contrario
short processDataLaser(Modulo_Teleoperacion::msg_laser);
