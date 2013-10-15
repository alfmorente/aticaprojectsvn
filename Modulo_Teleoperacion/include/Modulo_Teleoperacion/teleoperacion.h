/* 
 * File:   teleoperacion.h
 * Author: carlosamores
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
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_com_teleoperado.h"
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_errores.h"
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_habilitacion_modulo.h"
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_laser.h"
#include "../../msg_gen/cpp/include/Modulo_Teleoperacion/msg_modo.h"

//ROS y demás librerias
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include "constant.h"

//Funciones de suscripcion
void fcn_sub_com_teleop(const Modulo_Teleoperacion::msg_com_teleoperado);
void fcn_sub_hab_modulos(const Modulo_Teleoperacion::msg_habilitacion_modulo);
void fcn_sub_laser(const Modulo_Teleoperacion::msg_laser);

//Funciones propias
int convertToCorrectValues(int,int);
// Devuelve true si el proccesado de datos dictamina peligro y false en caso contrario
bool processDataLaser(Modulo_Teleoperacion::msg_laser);
