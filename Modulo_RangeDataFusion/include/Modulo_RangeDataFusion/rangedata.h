/* 
 * File:   rangedata.h
 * Author: atica
 *
 * Created on 18 de marzo de 2014, 12:23
 */

#ifndef RANGEDATA_H
#define	RANGEDATA_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* RANGEDATA_H */

// Mensajes
#include "Common_files/msg_gps.h"
//#include "Common_files/msg_beacon.h"
#include "Common_files/msg_laser.h"
#include "Common_files/msg_rangedatafusion.h"

// Funciones de suscripcion
void fcn_sub_gps(const Common_files::msg_gps);
void fcn_sub_laser(const Common_files::msg_laser);
//void fcn_sub_beacon(const Common_files::msg_beacon);

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