/* 
 * File:   teachMapping.h
 * Author: carlosamores
 *
 * Created on 16 de septiembre de 2013, 12:00
 */

#ifndef TEACHMAPPING_H
#define	TEACHMAPPING_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* TEACHMAPPING_H */

// Mensajes
#include "../../msg_gen/cpp/include/Modulo_TeachMapping/msg_errores.h"
#include "../../msg_gen/cpp/include/Modulo_TeachMapping/msg_gps.h"
#include "../../msg_gen/cpp/include/Modulo_TeachMapping/msg_habilitacion_modulo.h"
#include "../../msg_gen/cpp/include/Modulo_TeachMapping/msg_laser.h"

// ROS
#include "ros/ros.h"
#include "constant.h"
#include <queue>
#include <iostream>
#include <vector>
#include <fstream>

//Funciones de suscripcion
void fcn_sub_hab_modulos(const Modulo_TeachMapping::msg_habilitacion_modulo);
void fcn_sub_gps(const Modulo_TeachMapping::msg_gps);
void fcn_sub_laser(const Modulo_TeachMapping::msg_laser);

// Funciones propias
bool saveDataLaser();
bool saveDataGPS();

