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

// Constantes de modo de operacion del software
#define OPERATION_MODE_DEBUG 1
#define OPERATION_MODE_RELEASE 2
#define OPERATION_MODE_SIMULATION 3

// Constantes de gestion de cada modulo
#define STATE_OFF 0
#define STATE_CONF 1
#define STATE_OK 2
#define STATE_ERROR 3

// Constante para gestión de interruptores/banderas
#define OFF 0
#define ON 1

//Constantes para definir tipo de mensajes (Mensajes que requieran ACK)
#define SET 0
#define INFO 1

// Constantes de tipos de parada de emergencia
#define TOS_OBSTACLE 0
#define TOS_REMOTE 1
#define TOS_MANUAL 2

// Modos de operacion - msg_modo
#define MODE_NEUTRAL -1
#define MODE_MANUAL 0
#define MODE_REMOTE 1
#define MODE_PLAN 2
#define MODE_COME_TO_ME 3
#define MODE_FOLLOW_ME 4
#define MODE_CONVOY 5
#define MODE_CONVOY_TELEOP 6
#define MODE_CONVOY_AUTO 7

// Estado del Modo de operación
#define MODE_START 0
#define MODE_STOP 1
#define MODE_RUN 2
#define MODE_EXIT 3
#define MODE_FINISH 4

// Subsistemas - id_subsistema de msg_errores
#define SUBS_COMMUNICATION 0
#define SUBS_SYSTEM_MGMNT 1
#define SUBS_REMOTE 2
#define SUBS_DRIVING 3
#define SUBS_NAVIGATION 4
#define SUBS_CAMERA 5
#define SUBS_GPS 6
#define SUBS_FRONT_LASER_1 7
#define SUBS_FRONT_LASER_2 8
#define SUBS_REAR_LASER 9
#define SUBS_LASER_3D 10
#define SUBS_BEACON 11
#define SUBS_RANGE_DATA_FUSION 12
#define SUBS_HUMAN_LOCALIZATION 13
#define SUBS_CONVOY 14

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
#define FUNCTION_NOT_AVAILABLE 1
#define COMM_MODULE_NA 2
#define REMOTE_MODULE_NA 3
#define DRIVING_MODULE_NA 4
#define NAVIGATION_MODULE_NA 5
#define CAMERA_MODULE_NA 6
#define GPS_MODULE_NA 7
#define FRONT_LASER_MODULE_NA 8
#define REAR_LASER_MODULE_NA 9
#define LASER3D_MODULE_NA 10
#define BEACON_MODULE_NA 11
#define RDF_MODULE_NA 12
#define HL_MODULE_NA 13
#define CONVOY_MODULE_NA 14
// Output Errors
#define MODE_OR_FUNCTION_NA 100
#define MODULE_NOT_AVAILABLE 101

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
// Output Errors
#define CAN_FAILURE 100
#define MECHANICAL_FAILURE 101

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
// Output Errors
#define DATA_RCV_FAILED_OUT 100
#define SERIALPORT_ERROR 101
#define GPS_CONFIG_ERROR 102
#define IMU_ERROR 103
#define GPS_RECEIVER_ERROR 104

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
// Output Errors
#define LASER_CONNECTION_ERROR 100
#define LASER_COMMUNICATION_ERROR 101
#define LASER_CONFIGURATION_ERROR 102
#define LASER_COMMAND_ERROR 103
#define LASER_FRAME_ERROR 104
#define LASER_FILE_ERROR 105

// Subsystem = Communications
#define JAUS_CONFIG_ERROR 0
#define CREATE_COMPONENT_ERROR 1
#define RUN_COMPONENT_ERROR 2
#define COMM_LOG_FILE_ERROR 3
#define COMM_CONFIG_FILE_ERROR 4
#define COMM_CONFIG_FILE_STRUCTURE_ERROR 5
// Output Errors
#define JAUS_ERROR 100
#define COMM_FILE_ERROR 101

// Subsystem = Camera
#define ERROR_CAMERA_CONFIGURATION 1
#define ERROR_CAMERA_COMMUNICATION 2
// Output Errors
#define CAMERA_ERROR 100

// Subsystem = Navigation
#define ERROR_NAVIGATION_OBSTACLE_UNAVOIDABLE 1
#define ERROR_NAVIGATION_WAYPOINTS_GETTING_TIMEOUT 2
// Output Errors
#define NAV_ERROR 100

// Subsystem = Remote
#define REMOTE_PARAMETER_OUTRANGE 0
// Output
#define REMOTE_PARAMETER_ERROR 100

// Subsystem = RangeDataFusion
#define RDF_INSUFFICIENT_DATA 0
// Output
#define RDF_INSUFFICIENT_DATA_OUT 100

// Subsystem = HumanLocalization
#define HL_INSUFFICIENT_DATA 0
// Output
#define HL_INSUFFICIENT_DATA_OUT 100

// Subsystem = Convoy
#define UDP_SOCKET_FAIL 0
#define BIND_SOCKET_FAIL 1
#define SOCKET_RECEIVE_ERROR 2
#define LEADER_CRITICAL_ERROR 3
#define FOLLOWER_GPS_ERROR 4
#define FOLLOWER_LASER_ERROR 5
#define FOLLOWER_NAVIGATION_ERROR 6
#define FOLLOWER_CAMERA_ERROR 7
// Output Errors
#define CONVOY_CONNECTION_ERROR 100
#define CONVOY_SOCKET_RECEIVE_ERROR 101
#define LEADER_CRITICAL_ERROR_OUTPUT 102
#define FOLLOWER_ERROR 103

/**************************************
 ************** END ERRORS ************
 *************************************/


// ID Modulo - msg_module_enable
#define ID_MOD_REMOTE 0
#define ID_MOD_NAVIGATION 1
#define ID_MOD_CONVOY 2
#define ID_MOD_MAPPING 3
#define ID_MOD_TEACH 4

// Submodos - msg_module_enable
#define SUBMODE_REMOTE 0
#define SUBMODE_NAV_PLAN 0
#define SUBMODE_NAV_FOLLOW_ME 1
#define SUBMODE_NAV_COME_TO_ME 2
#define SUBMODE_NAV_CONVOY 3
#define SUBMODE_MAPPING 0
#define SUBMODE_TEACH 0
#define SUBMODE_CONVOY 0

// Estados - msg_module_enable
#define MOD_OFF 0
#define MOD_ON 1
#define MOD_PAUSE 2

// Elementos de los actuadores (msg_com_teleop)
#define ID_REMOTE_THROTTLE 0
#define ID_REMOTE_BRAKE 1
#define ID_REMOTE_STEER 2
#define ID_REMOTE_GEAR 3
#define ID_REMOTE_HANDBRAKE 4
#define ID_REMOTE_ENGINE 5
#define ID_REMOTE_LIGHT_IR 6
#define ID_REMOTE_LIGHT_CONVENTIONAL 7
#define ID_REMOTE_DIFF 8
#define ID_REMOTE_ACT_LASER2D 9

//Valores para marcha (msg_com_teleop)
#define GEAR_HIGH  0
#define GEAR_NEUTRAL_HIGH 1
#define GEAR_REVERSE 2
#define GEAR_NEUTRAL_LOW 3
#define GEAR_LOW 4

// Control de la camara (msg_ctrl_camera)
#define CAMERA_PAN 0
#define CAMERA_TILT 1
#define CAMERA_ZOOM 2
#define CAMERA_HOME 3

// Control de la camara (msg_ctrl_camera)
#define CAMERA_PAN_RIGHT 0
#define CAMERA_PAN_LEFT 1
#define CAMERA_PAN_STOP 2
#define CAMERA_TILT_UP 0
#define CAMERA_TILT_DOWN 1
#define CAMERA_TILT_STOP 2
#define CAMERA_ZOOM_IN 0
#define CAMERA_ZOOM_OUT 1
#define CAMERA_ZOOM_STOP 2
#define CAMERA_HOME_VALUE 0

// Identificador de lase (msg_laser)
#define ID_FRONT_LASER_1 0
#define ID_FRONT_LASER_2 1
#define ID_REAR_LASER 2

// Control de la navegacion
// TODO

// Tipo de fichero (msg_stream)
#define TOF_PLAN 0
#define TOF_TEACH 1
#define TOF_CONFIGURATION 2

// Posiciones de modos y funciones dentro del vector de msg_available
#define AVAILABLE_POS_REMOTE 1
#define AVAILABLE_POS_PLAN 2
#define AVAILABLE_POS_COME_TO_ME 3
#define AVAILABLE_POS_FOLLOW_ME 4
#define AVAILABLE_POS_CONVOY 5
#define AVAILABLE_POS_CONVOY_TELEOP 6
#define AVAILABLE_POS_CONVOY_AUTO 7
#define AVAILABLE_POS_START_ENGINE 8
#define AVAILABLE_POS_STOP_ENGINE 9
#define AVAILABLE_POS_ENGAGE_BRAKE 10
#define AVAILABLE_POS_MAPPING 11
#define AVAILABLE_POS_TEACH 12

//Define las funciones especiales
#define ENGINE 0
#define BRAKE  1
#define TEACH  2
#define MAPPING 3

//Define la posición del conmutador
#define MANUAL 0
#define TELEOP 1

//Define parametros del servidor de datos
#define PARAM_MODE 0