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

// Subsistemas - id_subsistema de msg_errores
#define SUBS_COMUNICACIONES 0
#define SUBS_GEST_SISTEMA 1
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

/**************************************
 ************** ERRORS **************
 ************************************/
// TYPE OF ERROR
#define TOE_UNDEFINED 0
#define TOE_WARNING 1
#define TOE_CRITICAL 2
#define TOE_END_ERROR 3

// ID ERROR

// General
#define ERROR_MODULE_UNAVAILABLE 0

// Subsystem = System Management
#define ERROR_MODE_GPS  1
#define ERROR_MODE_LASER 2
#define ERROR_MODE_DRIVING 3
#define ERROR_MODE_NAVIGATION 4

// Subsystem = Driving
#define ERROR_DRIVING_HANDBRAKE 1
#define ERROR_DRIVING_GEAR 2
#define ERROR_DRIVING_ENGINE 3
#define ERROR_DRIVING_ROBOT 4

// Subsystem = GPS
#define ERROR_GPS_NOT_SATELLITE_SIGNAL 1
#define ERROR_GPS_SOLUTION_NOT_GOOD 2
#define ERROR_GPS_MEMORY_OVERFLOW_TEACH 3
#define ERROR_GPS_IMU_BAD_ALIGNMENT 4

// Subsystem = Laser 2D
#define ERROR_LASER_CONFIGURATION 1
#define ERROR_LASER_COMMUNICATION 2

// Subsystem = Communications
#define ERROR_COMM_CONNECTION 1
#define ERROR_COMM_CONFIGURATION 2

// Subsystem = Camera
#define ERROR_CAMERA_CONFIGURATION 1
#define ERROR_CAMERA_COMMUNICATION 2

// Subsystem = Navigation
#define ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE 1
#define ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT 2

// Subsystem = Teleoperate
#define ERROR_TELEOP_NEAR_OBSTACLE 1
#define ERROR_TELEOP_FAR_OBSTABLE 2