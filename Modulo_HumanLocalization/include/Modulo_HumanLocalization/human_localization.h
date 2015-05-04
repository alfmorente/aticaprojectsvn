/** 
 * @file  human_localization.h
 * @brief Archivo de cabecera del módulo Human Localization
 * @author Alfonso Morente
 * @date 10/12/2013
 * @addtogroup HumanLocalization
 * @{
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
#include "Common_files/msg_error.h"
#include "Common_files/msg_waypoint.h"
#include "Common_files/msg_module_enable.h"
#include "Common_files/msg_rangedatafusion.h"
#include "../../../Common_files/srv_gen/cpp/include/Common_files/srv_data.h"


// Funciones de suscripcion
void fcn_sub_waypoint(const Common_files::msg_waypoint);
void fcn_sub_enableModule(const Common_files::msg_module_enable);
void fcn_sub_rangedatafusion(const Common_files::msg_rangedatafusion);
bool fcn_heartbeat(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp);

//ROS y demás librerias
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <pwd.h>
#include "../../../Common_files/include/Common_files/constant.h"
#include "interaction.h"

/**
 *@}
 */