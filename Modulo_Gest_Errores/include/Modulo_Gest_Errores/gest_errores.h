/* 
 * File:   gest_errores.h
 * Author: atica
 *
 * Created on 13 de septiembre de 2013, 11:07
 */


#ifndef GEST_ERRORES_H
#define	GEST_ERRORES_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* GEST_ERRORES_H */

// Mensajes
#include "Common_files/msg_mode.h"
#include "Common_files/msg_available.h"
#include "Common_files/msg_error.h"

//ROS y demás librerias
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/time.h>
#include <pwd.h>
#include "../../../Common_files/include/Common_files/constant.h"
#include <signal.h>
#include "interaction.h"

// Definición de variables globales
// Variable de control de modo
short currentMode;
// Variable de continuacion de modulo
bool exitModule;
// Variable control modos disponibles
Common_files::msg_available avail_mode;
// Variable con el número de errores por modo
int numErrorMode[13][15][28];

// Funciones de suscripcion
void fcn_sub_modo(const Common_files::msg_mode);
void fcn_sub_errores(const Common_files::msg_error);

// Funciones propias
void initialize(ros::NodeHandle n);
short isWarningOrCritical(Common_files::msg_error, short modo);
void switchNeutral();
int convertOutputError(Common_files::msg_error);
short mode_remote_error(Common_files::msg_error);
short mode_startengine_error(Common_files::msg_error);
short mode_stopengine_error(Common_files::msg_error);
short mode_engagebrake_error(Common_files::msg_error);
short mode_plan_error(Common_files::msg_error);
short mode_cometome_error(Common_files::msg_error);
short mode_followme_error(Common_files::msg_error);
short mode_teach_error(Common_files::msg_error);
short mode_mapping_error(Common_files::msg_error);
short mode_convoy_error(Common_files::msg_error);
short mode_conv_teleop_error(Common_files::msg_error);
short mode_conv_auto_error(Common_files::msg_error);
void writeToLog(Common_files::msg_error);
void updateModeAvailable (Common_files::msg_error);
void updateEndError(Common_files::msg_error);

// Definicion constantes propias del modulo
#define MODE_START_ENGINE 8
#define MODE_STOP_ENGINE 9
#define MODE_ENGAGE_BRAKE 10
#define MODE_MAPPING 11
#define MODE_TEACH 12

#define TOE_UNAVAILABLE 99
