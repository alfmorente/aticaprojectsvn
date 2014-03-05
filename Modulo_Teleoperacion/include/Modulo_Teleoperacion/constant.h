/* 
 * File:   constant.h
 * Author: atica
 *
 * Created on 19 de septiembre de 2013, 12:00
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef	__cplusplus
}
#endif

#endif	/* CONSTANT_H */
// Valor máximo de errores de fuera de rango de los comandos
#define MAX_OUTRANGE_ERROR 10

// Constantes del fichero de configuracion
#define STATE_OFF 0
#define STATE_CONF 1
#define STATE_OK 2
#define STATE_ERROR 3

// TYPE OF ERROR - msg_error
#define TOE_UNDEFINED 0
#define TOE_WARNING 1
#define TOE_CRITICAL 2
#define TOE_END_ERROR 3

// Subsistemas - id_subsistema de msg_error
#define SUBS_COMUNICACIONES 0
#define SUBS_GEST_SUSTEMA 1
#define SUBS_TELEOP 2
#define SUBS_CONDUCCION 3
#define SUBS_NAVEGACION 4
#define SUBS_CAMARA 5
#define SUBS_GPS 6
#define SUBS_LASER_DELANTERO 7
#define SUBS_LASER_TRAS_IZQ 8
#define SUBS_LASER_TRAS_DER 9
#define SUBS_LASER_3D 10

// Identificador del error - msg_error
#define REMOTE_PARAMATER_OUTRANGE 0
#define NEAR_OBSTACLE_DETECTION 1
#define FAR_OBSTACLE_DETECTION 2

// Modos de operacion - msg_modo
#define MODE_NEUTRAL -1
#define MODE_TELEOP 0
#define MODE_START_ENGINE 1
#define MODE_STOP_ENGINE 2
#define MODE_ENGAGE_BRAKE 3
#define MODE_PLAN 4
#define MODE_COME_TO_ME 5
#define MODE_FOLLOW_ME 6
#define MODE_MAPPING 7
#define MODE_TEACH 8
#define MODE_CONVOY 9
#define MODE_CONVOY_TELEOP 10
#define MODE_CONVOY_AUTO 11

// Estado del Modo de operación
#define MODE_START 0
#define MODE_STOP 1
#define MODE_RUN 2
#define MODE_EXIT 3
#define MODE_FINISH 4

// ID Modulo - msg_hab_modulo
#define ID_MOD_TELEOP 0
#define ID_MOD_NAVEGACION 1
#define ID_MOD_MAPPING 2
#define ID_MOD_TEACH 3

// Submodos - msg_hab_modulo
#define SUBMODE_TELEOP_TELEOP 0
#define SUBMODE_TELEOP_START_ENGINE 1
#define SUBMODE_TELEOP_STOP_ENGINE 2
#define SUBMODE_TELEOP_ENGAGE_BREAK 3
#define SUBMODE_NAV_PLAN 0
#define SUBMODE_NAV_FOLLOW_ME 1
#define SUBMODE_NAV_COME_TO_ME 2
#define SUBMODE_MAPPING_UNDFD 0
#define SUBMODE_TEACH_UNDFD 0

// Elementos de los actuadores (msg_com_teleop)
#define ID_TELEOP_THROTTLE 0
#define ID_TELEOP_BRAKE 1
#define ID_TELEOP_STEER 2
#define ID_TELEOP_GEAR 3
#define ID_TELEOP_HANDBRAKE 4
#define ID_TELEOP_ENGINE 5
#define ID_TELEOP_LIGHTS 6
#define ID_TELEOP_IR_LIGHTS 7
#define ID_TELEOP_DIFF 8
#define ID_TELEOP_LASER 9


// Cotas Maximas y minimas para actuadores (depuracion msg_com_teleop)
#define MAX_STEER_VALUE 100 // Giro maximo
#define MIN_STEER_VALUE -100 // Giro minimo
#define MAX_THROTTLE_VALUE 100 // Ac maximo
#define MIN_THROTTLE_VALUE 0 // Ac minimo
#define MAX_BRAKE_VALUE 100 // Freno maximo
#define MIN_BRAKE_VALUE 0 // Freno minimo
#define MAX_HANDBRAKE_VALUE 1 // Freno de mano puesto
#define MIN_HANDBRAKE_VALUE 0 // Freno de mano quitado
#define MAX_GEAR_VALUE 4 // Marcha maxima
#define MIN_GEAR_VALUE 0 // Marcha minima
#define MAX_LIGHTS_VALUE 1 // Luces puestas
#define MIN_LIGHTS_VALUE 0 // Luces quitadas
#define MAX_LIGHTS_IR_VALUE 1 // Luces puestas
#define MIN_LIGHTS_IR_VALUE 0 // Luces quitadas
#define MAX_ENGINE_VALUE 1 // Motor encendido
#define MIN_ENGINE_VALUE 0 // Motor apagado
#define MAX_DIFF_VALUE 1 // Diferencial ON
#define MIN_DIFF_VALUE 0 // Diferencial apagado
#define MAX_LASER_VALUE 1 // Activación laser
#define MIN_LASER_VALUE 0 // Desactivación laser

// Valores de los comandos
#define STOP_ENGINE 0
#define START_ENGINE 1
#define HANDBRAKE_OFF 0
#define HANDBRAKE_ON 1

// Valores de la variable obstacle en subscriptor laser
#define NO_OBSTACLE 0
#define FAR_OBSTACLE 1
#define NEAR_OBSTACLE 2
