/* 
 * File:   human_localization.h
 * Author: atica
 *
 * Created on 10 de diciembre de 2013, 12:42
 */

#ifndef HUMAN_LOCALIZATION_H
#define	HUMAN_LOCALIZATION_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif
#endif	/* HUMAN_LOCALIZATION_H */

// Mensajes
#include "../../msg_gen/cpp/include/Modulo_HumanLocalization/msg_errores.h"
#include "../../msg_gen/cpp/include/Modulo_HumanLocalization/msg_waypoint.h"
#include "../../msg_gen/cpp/include/Modulo_HumanLocalization/msg_module_enable.h"

// Funciones de suscripcion
void fcn_sub_waypoint(const Modulo_HumanLocalization::msg_waypoint);
void fcn_sub_enableModule(const Modulo_HumanLocalization::msg_module_enable);

//ROS y demás librerias
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <pwd.h>
#include "constant.h"
