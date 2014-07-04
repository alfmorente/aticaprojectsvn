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

// Stado del Modo de operación
#define MODE_START 0
#define MODE_STOP 1
#define MODE_RUN 2
#define MODE_EXIT 3
#define MODE_FINISH 4

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

// Tipo de mensaje ROS (para traducir a JAUS y viceversa)
#define TOM_CAMERA 0
#define TOM_GPS 1
#define TOM_ERROR 2
#define TOM_MODE 3
#define TOM_WAYPOINT 4
#define TOM_PLAN 5
#define TOM_TELEOP 6
#define TOM_BACKUP 7
#define TOM_ACK 8
#define TOM_PET_CONFIGURATION 9
#define TOM_CONFIGURATION 10
#define TOM_AVAILABLE_MODE 11
#define TOM_UNKNOW -1



// Elementos de los actuadores (msg_com_teleoperado)
#define ID_TELEOP_THROTTLE 0
#define ID_TELEOP_BRAKE 1
#define ID_TELEOP_STEER 2
#define ID_TELEOP_GEAR 3
#define ID_TELEOP_HANDBRAKE 4
#define ID_TELEOP_ENGINE 5
#define ID_TELEOP_LIGHT_IR 6
#define ID_TELEOP_LIGHT_STANDAR 7
#define ID_TELEOP_DIFF 8
#define ID_TELEOP_ACT_LASER2D 9


// Cotas Maximas y minimas para actuadores (depuracion msg_com_teleoperado)
#define MAX_STEER_VALUE 100 // Giro maximo
#define MIN_STEER_VALUE -100 // Giro minimo
#define MAX_THROTTLE_VALUE 100 // Acc maximo
#define MIN_THROTTLE_VALUE 0 // Acc minimo
#define MAX_BRAKE_VALUE 100 // Freno maximo
#define MIN_BRAKE_VALUE 0 // Freno minimo
#define MAX_HANDBRAKE_VALUE 1 // Freno de mano puesto
#define MIN_HANDBRAKE_VALUE 0 // Freno de mano quitado
#define MAX_GEAR_VALUE 2 // Marcha maxima
#define MIN_GEAR_VALUE 0 // Marcha minima
#define MAX_LIGHTS_VALUE 1 // Luces puestas
#define MIN_LIGHTS_VALUE 0 // Luces quitadas
#define MAX_ENGINE_VALUE 1 // Motor encendido
#define MIN_ENGINE_VALUE 0 // Motor apagado

//Valores para otros actuadores
#define HIGH  0
#define NEUTRAL_HIGH 1
#define REVERSE 2
#define NEUTRAL_LOW 3
#define LOW 4

#define ON 1
#define OFF 0
#define POSITION 1
#define CROSSING 2
#define BEAM 3

//Valores JAUS
#define JAUS_HIGH 25
#define JAUS_NEUTRAL_HIGH 76
#define JAUS_REVERSE 127
#define JAUS_NEUTRAL_LOW 178
#define JAUS_LOW 229

#define MISSION 0
#define TASK 1
#define SPOOLING 0
#define PENDING 1
#define PAUSED 2
#define ABORTED 3
#define FINISHED 4

