/* 
 * File:   gest_navegacion.h
 * Author: carlosamores
 *
 * Created on 13 de septiembre de 2013, 12:27
 */

#ifndef GEST_NAVEGACION_H
#define	GEST_NAVEGACION_H

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* GEST_NAVEGACION_H */

//Mensajes
#include "../../msg_gen/cpp/include/Modulo_Navegacion/msg_error.h"
#include "../../msg_gen/cpp/include/Modulo_Navegacion/msg_module_enable.h"
#include "../../msg_gen/cpp/include/Modulo_Navegacion/msg_gps.h"
#include "../../msg_gen/cpp/include/Modulo_Navegacion/msg_gest_navegacion.h"
#include "../../msg_gen/cpp/include/Modulo_Navegacion/msg_laser.h"
#include "../../msg_gen/cpp/include/Modulo_Navegacion/msg_mode.h"
#include "../../msg_gen/cpp/include/Modulo_Navegacion/msg_waypoints.h"

//ROS y demas librerias
#include "ros/ros.h"
#include "constant.h"
#include <queue>
#include <iostream>
#include <vector>


//Estructura que contiene los datos de los sensores
typedef struct{
    float lat;
    float lon;
    float alt;
    float pitch;
    float yaw;
    float roll;
    std::vector<float> angles;
    std::vector<float> distances;
}Sensors;

typedef struct{

    short submodule;
    short status;

}mode_nav;


//Funciones de suscripcion
void fcn_sub_gps(const Modulo_Navegacion::msg_gps);
void fcn_sub_laser(const Modulo_Navegacion::msg_laser);
void fcn_sub_waypoints(const Modulo_Navegacion::msg_waypoints);
void fcn_sub_module_enable(const Modulo_Navegacion::msg_module_enable);

// Funciones propias
// Transforma a mensaje necesario para move_base el valor de los sensores
Modulo_Navegacion::msg_gest_navegacion adaptSensorValues(Sensors);
// Calcula la distancia minima para considerar si se ha llegado al WP actual
bool hasReachedToWP(float,float, float, float );
bool checkEndListWaypoints();

//Se encarga de la navegacion del vehiculo en los tres submodos posibles
void fcn_mng_CTME();
void fcn_mng_PLAN();
void fcn_mng_FLME();