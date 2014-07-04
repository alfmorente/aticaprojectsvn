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
#define SUBS_BEACON 11
#define SUBS_RDF 12
#define SUBS_HL 13
#define SUBS_CONVOY 14

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
// Subsystem = System Management
#define MODE_NOT_AVAILABLE 0
#define COMM_MODULE_NA 1
#define REMOTE_MODULE_NA 2
#define DRIVING_MODULE_NA 3
#define NAVIGATION_MODULE_NA 4
#define CAMERA_MODULE_NA 5
#define GPS_MODULE_NA 6
#define FRONT_LASER_MODULE_NA 7
#define REAR_LASER_MODULE_NA 8
#define LASER3D_MODULE_NA 9
#define BEACON_MODULE_NA 10
#define RDF_MODULE_NA 11
#define HL_MODULE_NA 12
#define CONVOY_MODULE_NA 13

// Subsystem = Driving
#define CONNECTION_CAN_FAIL 0
#define COMMUNICATION_CAN_FAIL 1
#define START_STOP_FAILURE 2
#define THROTTLE_FAILURE 3
#define HANDBRAKE_FAILURE 4
#define BRAKE_FAILURE 5
#define GEAR_SHIFT_FAILURE 6
#define STEER_FAILURE 7
#define DIFFERENTIAL_LOCK_FAILURE 8

// Subsystem = GPS
#define GPS_GLOBAL_ERROR 0
#define DATA_RCV_FAILED 1
#define CONFIG_OPEN_SERIALPORT_ERROR 2
#define CONFIG_CONFIG_SERIALPORT_ERROR 3
#define CONFIG_ALIGNMENT_MODE_ERROR 4
#define CONFIG_INITIAL_AZIMUT_ERROR 5
#define CONFIG_ANTOFFSET_ERROR 6
#define INS_INACTIVE 7
#define INS_ALIGNING 8
#define INS_SOLUTION_NOT_GOOD 9
#define INS_BAD_GPS_AGREEMENT 10
#define INS_ALIGNMENT_COMPLETE 11
#define INSUFFICIENT_OBS 12
#define NO_CONVERGENCE 13
#define SINGULARITY 14
#define COV_TRACE 15
#define TEST_DIST 16
#define COLD_START 17
#define V_H_LIMIT 18
#define VARIANCE 19
#define RESIDUALS 20
#define DELTA_POS 21
#define NEGATIVE_VAR 22
#define INTEGRITY_WARNING 23
#define IMU_UNPLUGGED 24
#define PENDING 25
#define INVALID_FIX 26
#define UNAUTHORIZED_STATE 27

// Subsystem = LASER 2D
#define LASER_SOCKET_FAIL 0
#define CONNECTION_ERROR 1
#define COMM_ERROR 2
#define INVALID_SAMPLE_FREQ 3
#define INVALID_ANGLE_RESOLUTION 4
#define INVALID_ANGLE_SAMPLE_RESOLUTION 5
#define INVALID_SCAN_AREA 6
#define UNKNOWN_ERROR 7
#define START_MEASURE_NA 8
#define STOP_MEASURE_NA 9
#define SELECT_USER_LEVEL_NA 10
#define SET_LMS_OUTPUT_NA 11
#define SAVE_DATA_NA 12
#define START_DEVICE_NA 13
#define INCORRECT_ANSWER 14
#define FRAME_OVERFLOW 15
#define LASER_LOG_FILE_ERROR 16
#define LASER_CONFIG_FILE_ERROR 17
#define LASER_CONFIG_FILE_STRUCTURE_ERROR 18

// Subsystem = Communications
#define LINK_ERROR_900 0
#define LINK_ERROR_2400 1
#define JAUS_CONFIG_ERROR 2
#define CREATE_COMPONENT_ERROR 3
#define RUN_COMPONENT_ERROR 4
#define COMM_LOG_FILE_ERROR 5
#define COMM_CONFIG_FILE_ERROR 6
#define COMM_CONFIG_FILE_STRUCTURE_ERROR 7

// Subsystem = Camera
#define ERROR_CAMERA_CONFIGURATION 1
#define ERROR_CAMERA_COMMUNICATION 2

// Subsystem = Navigation
#define ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE 1
#define ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT 2

// Subsystem = Remote
#define REMOTE_PARAMETER_OUTRANGE 0
#define NEAR_OBSTACLE_DETECTION 1
#define FAR_OBSTABLE_DETECTION 2

// Subsystem = RangeDataFusion
#define RDF_INSUFFICIENT_DATA 0

// Subsystem = HumanLocalization
#define HL_INSUFFICIENT_DATA 0

// Subsystem = Convoy
#define UDP_SOCKET_FAIL 0
#define BIND_SOCKET_FAIL 1
#define SOCKET_RECEIVE_ERROR 2
#define LEADER_CRITICAL_ERROR 3
#define FOLLOWER_GPS_ERROR 4
#define FOLLOWER_LASER_ERROR 5
#define FOLLOWER_NAVIGATION_ERROR 6
#define FOLLOWER_CAMERA_ERROR 7