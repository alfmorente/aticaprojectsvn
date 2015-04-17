/** 
 * @file  gest_errores.h
 * @brief Archivo de cabecera del módulo Gestión de errores
 * @author Alfonso Morente
 * @date 13/09/2013
 */

#ifndef GEST_ERRORES_H
#define	GEST_ERRORES_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

// Mensajes
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_mode.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_available.h"
#include "../../../Common_files/msg_gen/cpp/include/Common_files/msg_error.h"
#include "../../../Common_files/srv_gen/cpp/include/Common_files/srv_data.h"

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

// Definicion constantes propias del modulo
#define MODE_START_ENGINE 8     ///<Constante de modos no incluida en constant.h
#define MODE_STOP_ENGINE 9      ///<Constante de modos no incluida en constant.h
#define MODE_ENGAGE_BRAKE 10    ///<Constante de modos no incluida en constant.h
#define MODE_MAPPING 11         ///<Constante de modos no incluida en constant.h
#define MODE_TEACH 12           ///<Constante de modos no incluida en constant.h

#define TOE_UNAVAILABLE 99      ///<Constante para modo erróneo
#define MIN_MODES 1             ///<Usado para los bucles for que recorren la tabla de errores
#define MAX_MODES 14            ///<Usado para los bucles for que recorren la tabla de errores
#define MAX_MODULES 15          ///<Usado para los bucles for que recorren la tabla de errores
#define MAX_ERRORS 28           ///<Usado para los bucles for que recorren la tabla de errores

// Definición de variables globales
// Publicadores y suscriptores
ros::Publisher pub_mode;
ros::Publisher pub_error;
ros::Publisher pub_avail_mode;
ros::Subscriber sub_error;
ros::Subscriber sub_mode;
ros::ServiceServer server;
// Variable de control de modo
short currentMode;
// Variable de continuacion de modulo
bool exitModule;
// Variable control modos disponibles
Common_files::msg_availablePtr avail_mode(new Common_files::msg_available);
Common_files::msg_modePtr msg_ch_neutral(new Common_files::msg_mode);
// Variables con el número de errores por modo
int numErrorMode[MAX_MODES][MAX_MODULES][MAX_ERRORS];
int numWarning[MAX_MODES][MAX_MODULES][MAX_ERRORS];

// Funciones de suscripcion
void fcn_sub_mode(const Common_files::msg_modePtr&);
void fcn_sub_error(const Common_files::msg_errorPtr&);
bool fcn_heartbeat(Common_files::srv_data::Request &req, Common_files::srv_data::Response &resp);

// Funciones propias
void initialize(ros::NodeHandle n);
short isWarningOrCritical(const Common_files::msg_errorPtr&, short modo);
void switchNeutral();
int convertOutputError(const Common_files::msg_errorPtr&);
short mode_remote_error(const Common_files::msg_errorPtr&);
short mode_startengine_error(const Common_files::msg_errorPtr&);
short mode_stopengine_error(const Common_files::msg_errorPtr&);
short mode_engagebrake_error(const Common_files::msg_errorPtr&);
short mode_plan_error(const Common_files::msg_errorPtr&);
short mode_cometome_error(const Common_files::msg_errorPtr&);
short mode_followme_error(const Common_files::msg_errorPtr&);
short mode_teach_error(const Common_files::msg_errorPtr&);
short mode_mapping_error(const Common_files::msg_errorPtr&);
short mode_convoy_error(const Common_files::msg_errorPtr&);
short mode_conv_teleop_error(const Common_files::msg_errorPtr&);
short mode_conv_auto_error(const Common_files::msg_errorPtr&);
void writeToLog(const Common_files::msg_errorPtr&);
bool updateModeAvailable (const Common_files::msg_errorPtr&, bool[13]);
bool updateEndError(const Common_files::msg_errorPtr&, bool[13]);
int checkErrorTable(int);
bool compareTable(bool original[13],const Common_files::msg_availablePtr&);

#endif	/* GEST_ERRORES_H */

