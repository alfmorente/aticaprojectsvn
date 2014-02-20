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
#include "../../msg_gen/cpp/include/Modulo_Gest_Errores/msg_modo.h"
#include "../../msg_gen/cpp/include/Modulo_Gest_Errores/msg_com_teleoperado.h"
#include "../../msg_gen/cpp/include/Modulo_Gest_Errores/msg_errores.h"
#include "../../msg_gen/cpp/include/Modulo_Gest_Errores/msg_available_mode.h"
#include "../../msg_gen/cpp/include/Modulo_Gest_Errores/msg_confirm.h"

//ROS y demás librerias
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <pwd.h>
#include "constant.h"
#include <signal.h>

// Definición de variables globales
// Variable de control de modo
short modoActual;
// Variable de continuacion de modulo
bool exitModule;
// Variable control modos disponibles
Modulo_Gest_Errores::msg_available_mode avail_mode;
// Variable con el número de errores por modo
int num_err_mode[12];
// Bandera de confirmación de paso a modo neutro
bool confirm_flag;

// Funciones de suscripcion
void fcn_sub_modo(const Modulo_Gest_Errores::msg_modo);
void fcn_sub_errores(const Modulo_Gest_Errores::msg_errores);
void fcn_sub_confirm(const Modulo_Gest_Errores::msg_confirm);

// Funciones propias
short isWarningOrCritical(Modulo_Gest_Errores::msg_errores, short modo);
void switchNeutral();
short mode_remote_error(Modulo_Gest_Errores::msg_errores);
short mode_startengine_error(Modulo_Gest_Errores::msg_errores);
short mode_stopengine_error(Modulo_Gest_Errores::msg_errores);
short mode_engagebrake_error(Modulo_Gest_Errores::msg_errores);
short mode_plan_error(Modulo_Gest_Errores::msg_errores);
short mode_cometome_error(Modulo_Gest_Errores::msg_errores);
short mode_followme_error(Modulo_Gest_Errores::msg_errores);
short mode_teach_error(Modulo_Gest_Errores::msg_errores);
short mode_mapping_error(Modulo_Gest_Errores::msg_errores);
short mode_convoy_error(Modulo_Gest_Errores::msg_errores);
short mode_conv_teleop_error(Modulo_Gest_Errores::msg_errores);
short mode_conv_auto_error(Modulo_Gest_Errores::msg_errores);