
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de mensajes JAUS y
 * ROS en interior y exterior de la arquitectura.
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

/*******************************************************************************
 * PROTOCOLO PAYLOAD DE CONDUCCION             
 * IDENTIFICADOR DE INSTRUCCION
 *******************************************************************************/

#define SET 0
#define GET 1
#define INFO 2

/*******************************************************************************
 * PROTOCOLO PAYLOAD DE CONDUCCION             
 * IDENTIFICADOR DE ELEMENTO
 *******************************************************************************/

#define RESET 0
#define BLINKER_RIGHT 1
#define BLINKER_LEFT 2
#define BLINKER_EMERGENCY 3
#define MT_BLINKERS 4
#define DIPSP 5
#define DIPSS 6
#define DIPSR 7
#define KLAXON 8
#define MT_LIGHTS 9
#define GEAR 10
#define MT_GEAR 11
#define THROTTLE 12
#define MOTOR_RPM 13
#define CRUISING_SPEED 14 
#define MT_THROTTLE 15
#define MOTOR_TEMPERATURE 16
#define HANDBRAKE 17
#define MT_HANDBRAKE 18
#define BRAKE 19
#define MT_BRAKE 20
#define STEERING 21
#define STEERING_ALARMS 22
#define MT_STEERING 23
#define DRIVE_ALARMS 24
#define BATTERY_LEVEL 25
#define BATTERY_VOLTAGE 26
#define BATTERY_CURRENT 27
#define BATTERY_TEMPERATURE 28
#define SUPPLY_TURN_ON 29
#define SUPPLY_CHECK 30
#define TURN_OFF 31
#define SUPPLY_5 32
#define SUPPLY_12 33
#define SUPPLY_24_DRIVE 34
#define SUPPLY_24_OCC 35
#define CONTROL_SYSTEM_SUPPLY 36
#define CONTROL_SYSTEM_SUPPLY_5 37
#define CONTROL_SYSTEM_SUPPLY_12 38
#define CONTROL_SYSTEM_SUPPLY_24 39
#define CONTROL_SYSTEM_SUPPLY_48 40
#define DRIVE_SYSTEM_SUPPLY 41
#define DRIVE_SYSTEM_SUPPLY_5 42
#define DRIVE_SYSTEM_SUPPLY_12 43
#define DRIVE_SYSTEM_SUPPLY_24 44
#define DRIVE_SYSTEM_SUPPLY_48 45
#define COMM_SYSTEM_SUPPLY 46
#define COMM_SYSTEM_SUPPLY_5 47
#define COMM_SYSTEM_SUPPLY_12 48
#define COMM_SYSTEM_SUPPLY_24 49
#define COMM_SYSTEM_SUPPLY_48 50
#define OBSERVATION_SYSTEM_SUPPLY 51
#define OBSERVATION_SYSTEM_SUPPLY_5 52
#define OBSERVATION_SYSTEM_SUPPLY_12 53
#define OBSERVATION_SYSTEM_SUPPLY_24 54
#define OBSERVATION_SYSTEM_SUPPLY_48 55
#define SUPPLY_ALARMS 56

/*******************************************************************************
 *                              ESTADOS DEL NODO
 *******************************************************************************/

#define NODESTATUS_INIT 0
#define NODESTATUS_OK 1
#define NODESTATUS_CORRUPT 2
#define NODESTATUS_OFF 3

/*******************************************************************************
 *                          MODOS DE OPERACION DEL VEHICULO
 *******************************************************************************/

#define OPERATION_MODE_LOCAL 0
#define OPERATION_MODE_INICIANDO 1
#define OPERATION_MODE_CONDUCCION 2
#define OPERATION_MODE_OBSERVACION 3
#define OPERATION_MODE_APAGANDO 4

/*******************************************************************************
 * FRECUENCIA -> PERIODO
 ******************************************************************************/

#define FREC_10HZ 0.1
#define FREC_5HZ 0.2
#define FREC_2HZ 0.5
#define FREC_1HZ 1

/*******************************************************************************
 *           MANEJO DE PRESENCE VECTOR
 *******************************************************************************/

// Set/Report Wrench Effort
#define PRESENCE_VECTOR_THROTTLE 0x0001
#define PRESENCE_VECTOR_STEER 0x0010
#define PRESENCE_VECTOR_BRAKE 0x0040

// Set/Report Discrete devices
#define PRESENCE_VECTOR_GEAR 0x04
#define PRESENCE_VECTOR_PARKING_BRAKE 0x02

// Report Camera Pose
#define PRESENCE_VECTOR_CURRENT_PAN 0x0040
#define PRESENCE_VECTOR_CURRENT_TILT 0x0020
#define PRESENCE_VECTOR_CURRENT_ZOOM 0x0008

// Set Camera Pose
#define PRESENCE_VECTOR_PAN 0x20
#define PRESENCE_VECTOR_TILT 0x10
#define PRESENCE_VECTOR_ZOOM 0x04

// UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_LEVEL 0x01
#define PRESENCE_VECTOR_BATTERY_VOLTAGE 0x02
#define PRESENCE_VECTOR_BATTERY_CURRENT 0x04
#define PRESENCE_VECTOR_BATTERY_TEMPERATURE 0x08
#define PRESENCE_VECTOR_MOTOR_TEMPERATURE 0x10
#define PRESENCE_VECTOR_MOTOR_RPM 0x20
#define PRESENCE_VECTOR_ALARMS 0x40

// Report Velocity State
#define PRESENCE_VECTOR_CURRENT_CRUISING_SPEED 0x0008

// Report Signaling Elements
#define PRESENCE_VECTOR_BLINKER_RIGHT 0x10
#define PRESENCE_VECTOR_BLINKER_LEFT 0x80
#define PRESENCE_VECTOR_KLAXON 0x20
#define PRESENCE_VECTOR_DIPSP 0x02
#define PRESENCE_VECTOR_DIPSR 0x04
#define PRESENCE_VECTOR_DIPSS 0x01

/*******************************************************************************
 ********************************************************************************
 *                       ARTEFACTOS JAUS
 ********************************************************************************
 *******************************************************************************/

/*******************************************************************************
 *                        SUBSISTEMAS JAUS
 *******************************************************************************/
#define JAUS_SUBSYSTEM_MYC 1
#define JAUS_SUBSYSTEM_USV 2
#define JAUS_SUBSYSTEM_UGV 3

/*******************************************************************************
 *                           NODOS JAUS
 *******************************************************************************/
#define JAUS_NODE_CONTROL 1
#define JAUS_NODE_TABLET 2
#define JAUS_NODE_CAMERA 2
#define JAUS_NODE_COMM_MNG 3
#define JAUS_NODE_AOC 4

/*******************************************************************************
 *              IDENTIFICADOR DE CAMARA DE APOYO A LA CONDUCCION
 ******************************************************************************/
#define FRONT_CAMERA_ID 1
#define REAR_CAMERA_ID 2


#endif	/* CONSTANT_H */




