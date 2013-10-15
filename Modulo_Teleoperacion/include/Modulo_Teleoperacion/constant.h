/* 
 * File:   constant.h
 * Author: carlosamores
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

// Constantes del fichero de configuracion
#define STATE_OFF 0
#define STATE_CONF 1
#define STATE_OK 2
#define STATE_ERROR 3

// Tipos de errores/alarmas (Warning or critical) - msg_errores
#define ALARM_UNDEFINED 0
#define ALARM_WARNING 1
#define ALARM_CRITICAL 2
#define ALARM_EXIT 3

// Modos de operacion - msg_modo
#define MODE_NEUTRAL 0
#define MODE_TELEOP 1
#define MODE_START_ENGINE 2
#define MODE_STOP_ENGINE 3
#define MODE_ENGAGE_BRAKE 4
#define MODE_PLAN 5
#define MODE_COME_TO_ME 6
#define MODE_FOLLOW_ME 7
#define MODE_MAPPING 8
#define MODE_TEACH 9

// Subsistemas - id_subsistema de msg_errores
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

// Elementos de los actuadores (msg_com_teleoperado)
#define ID_TELEOP_STEER 0
#define ID_TELEOP_THROTTLE 1
#define ID_TELEOP_BRAKE 2
#define ID_TELEOP_HANDBRAKE 3
#define ID_TELEOP_GEAR 4
#define ID_TELEOP_LIGHTS 5
#define ID_TELEOP_ENGINE 6

// Cotas Maximas y minimas para actuadores (depuracion msg_com_teleoperado)
#define MAX_STEER_VALUE 100 // Giro maximo
#define MIN_STEER_VALUE -100 // Giro minimo
#define MAX_THROTTLE_VALUE 100 // Acc maximo
#define MIN_THROTTLE_VALUE 0 // Acc minimo
#define MAX_BRAKE_VALUE 100 // Freno maximo
#define MIN_BRAKE_VALUE 0 // Freno minimo
#define MAX_HANDBRAKE_VALUE 1 // Freno de mano puesto
#define MIN_HANDBRAKE_VALUE 0 // Freno de mano quitado
#define MAX_GEAR_VALUE 6 // Marcha maxima
#define MIN_GEAR_VALUE 0 // Marcha minima
#define MAX_LIGHTS_VALUE 1 // Luces puestas
#define MIN_LIGHTS_VALUE 0 // Luces quitadas
#define MAX_ENGINE_VALUE 1 // Motor encendido
#define MIN_ENGINE_VALUE 0 // Motor apagado


