/* 
 * File:   gest_errores.h
 * Author: carlosamores
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

//ROS y demás librerias
#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <pwd.h>
#include "constant.h"

// Funciones de suscripcion
void fcn_sub_modo(const Modulo_Gest_Errores::msg_modo);
void fcn_sub_errores(const Modulo_Gest_Errores::msg_errores);

// Funciones propias
short isWarningOrCritial(Modulo_Gest_Errores::msg_errores);
void switchNeutral();
