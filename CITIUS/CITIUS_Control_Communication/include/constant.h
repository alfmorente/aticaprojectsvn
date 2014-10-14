
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de mensajes JAUS y
 * ROS en interior y exterior de la arquitectura.
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

/*******************************************************************************
 * PROTOCOLO PAYLOAD DE CONDUCCION             
 * IDENTIFICADOR DE ELEMENTO
 *******************************************************************************/

#define RESET 0 /// Valor para formación de comando RESET
#define BLINKER_RIGHT 1 /// Valor para formación de comando BLINKER_RIGHT
#define BLINKER_LEFT 2 /// Valor para formación de comando BLINKER_LEFT
#define BLINKER_EMERGENCY 3 /// Valor para formación de comando BLINKER_EMERGENCY
#define MT_BLINKERS 4 /// Valor para formación de comando MT_BLINKERS
#define DIPSP 5 /// Valor para formación de comando DIPSP
#define DIPSS 6 /// Valor para formación de comando DIPSS
#define DIPSR 7 /// Valor para formación de comando DIPSR
#define KLAXON 8 /// Valor para formación de comando KLAXON
#define MT_LIGHTS 9 /// Valor para formación de comando MT_LIGHTS
#define GEAR 10 /// Valor para formación de comando GEAR
#define MT_GEAR 11 /// Valor para formación de comando MT_GEAR
#define THROTTLE 12 /// Valor para formación de comando THROTTLE
#define MOTOR_RPM 13 /// Valor para formación de comando MOTOR_RPM
#define CRUISING_SPEED 14 /// Valor para formación de comando CRUISING_SPEED
#define MT_THROTTLE 15 /// Valor para formación de comando MT_THROTTLE
#define MOTOR_TEMPERATURE 16 /// Valor para formación de comando MOTOR_TEMPERATURE
#define HANDBRAKE 17 /// Valor para formación de comando HANDBRAKE
#define MT_HANDBRAKE 18 /// Valor para formación de comando MT_HANDBRAKE
#define BRAKE 19 /// Valor para formación de comando BRAKE
#define MT_BRAKE 20 /// Valor para formación de comando MT_BRAKE
#define STEERING 21 /// Valor para formación de comando STEERING
#define STEERING_ALARMS 22 /// Valor para formación de comando STEERING_ALARMS
#define MT_STEERING 23 /// Valor para formación de comando MT_STEERING
#define DRIVE_ALARMS 24 /// Valor para formación de comando DRIVE_ALARMS
#define BATTERY_LEVEL 25 /// Valor para formación de comando BATTERY_LEVEL
#define BATTERY_VOLTAGE 26 /// Valor para formación de comando BATTERY_VOLTAGE
#define BATTERY_CURRENT 27 /// Valor para formación de comando BATTERY_CURRENT
#define BATTERY_TEMPERATURE 28 /// Valor para formación de comando BATTERY_TEMPERATURE
#define SUPPLY_TURN_ON 29 /// Valor para formación de comando SUPPLY_TURN_ON
#define SUPPLY_CHECK 30 /// Valor para formación de comando SUPPLY_CHECK
#define TURN_OFF 31 /// Valor para formación de comando TURN_OFF
#define SUPPLY_5 32 /// Valor para formación de comando SUPPLY_5
#define SUPPLY_12 33 /// Valor para formación de comando SUPPLY_12
#define SUPPLY_24_DRIVE 34 /// Valor para formación de comando SUPPLY_24_DRIVE
#define SUPPLY_24_OCC 35 /// Valor para formación de comando SUPPLY_24_OCC
#define CONTROL_SYSTEM_SUPPLY 36 /// Valor para formación de comando CONTROL_SYSTEM_SUPPLY
#define CONTROL_SYSTEM_SUPPLY_5 37 /// Valor para formación de comando CONTROL_SYSTEM_SUPPLY_5
#define CONTROL_SYSTEM_SUPPLY_12 38 /// Valor para formación de comando CONTROL_SYSTEM_SUPPLY_12
#define CONTROL_SYSTEM_SUPPLY_24 39 /// Valor para formación de comando CONTROL_SYSTEM_SUPPLY_24
#define CONTROL_SYSTEM_SUPPLY_48 40 /// Valor para formación de comando CONTROL_SYSTEM_SUPPLY_48
#define DRIVE_SYSTEM_SUPPLY 41 /// Valor para formación de comando DRIVE_SYSTEM_SUPPLY
#define DRIVE_SYSTEM_SUPPLY_5 42 /// Valor para formación de comando DRIVE_SYSTEM_SUPPLY_5
#define DRIVE_SYSTEM_SUPPLY_12 43 /// Valor para formación de comando DRIVE_SYSTEM_SUPPLY_12
#define DRIVE_SYSTEM_SUPPLY_24 44 /// Valor para formación de comando DRIVE_SYSTEM_SUPPLY_24
#define DRIVE_SYSTEM_SUPPLY_48 45 /// Valor para formación de comando DRIVE_SYSTEM_SUPPLY_48
#define COMM_SYSTEM_SUPPLY 46 /// Valor para formación de comando COMM_SYSTEM_SUPPLY
#define COMM_SYSTEM_SUPPLY_5 47 /// Valor para formación de comando COMM_SYSTEM_SUPPLY_5
#define COMM_SYSTEM_SUPPLY_12 48 /// Valor para formación de comando COMM_SYSTEM_SUPPLY_12
#define COMM_SYSTEM_SUPPLY_24 49 /// Valor para formación de comando COMM_SYSTEM_SUPPLY_24
#define COMM_SYSTEM_SUPPLY_48 50 /// Valor para formación de comando COMM_SYSTEM_SUPPLY_48
#define OBSERVATION_SYSTEM_SUPPLY 51 /// Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY
#define OBSERVATION_SYSTEM_SUPPLY_5 52 /// Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY_5
#define OBSERVATION_SYSTEM_SUPPLY_12 53 /// Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY_12
#define OBSERVATION_SYSTEM_SUPPLY_24 54 /// Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY_24
#define OBSERVATION_SYSTEM_SUPPLY_48 55 /// Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY_48
#define SUPPLY_ALARMS 56 /// Valor para formación de comando SUPPLY_ALARMS
#define OPERATION_MODE_SWITCH 57 /// Valor para formación de comando OPERATION_MODE_SWITCH

/*******************************************************************************
 *                              ESTADOS DEL NODO
 *******************************************************************************/

#define NODESTATUS_INIT 0 /// Identificador del estado INIT de la máquina de estados de nodo
#define NODESTATUS_OK 1 /// Identificador del estado OK de la máquina de estados de nodo
#define NODESTATUS_CORRUPT 2 /// Identificador del estado CORRUPT de la máquina de estados de nodo
#define NODESTATUS_OFF 3 /// Identificador del estado OFF de la máquina de estados de nodo

/*******************************************************************************
 *                          MODOS DE OPERACION DEL VEHICULO
 *******************************************************************************/

#define OPERATION_MODE_LOCAL 0 /// Identificador del modo de operación LOCAL de la máquina de estados del vehículo
#define OPERATION_MODE_INICIANDO 1 /// Identificador del modo de operación OPERATION_MODE_INICIANDO de la máquina de estados del vehículo
#define OPERATION_MODE_CONDUCCION 2 /// Identificador del modo de operación OPERATION_MODE_CONDUCCION de la máquina de estados del vehículo
#define OPERATION_MODE_OBSERVACION 3 /// Identificador del modo de operación OPERATION_MODE_OBSERVACION de la máquina de estados del vehículo
#define OPERATION_MODE_APAGANDO 4 /// Identificador del modo de operación OPERATION_MODE_APAGANDO de la máquina de estados del vehículo

/*******************************************************************************
 * FRECUENCIA -> PERIODO
 ******************************************************************************/

#define FREC_10HZ 0.1 /// Periodo para ejecución de rutinas con frecuencia 10Hz
#define FREC_5HZ 0.2 /// Periodo para ejecución de rutinas con frecuencia 5Hz
#define FREC_2HZ 0.5 /// Periodo para ejecución de rutinas con frecuencia 2Hz
#define FREC_1HZ 1 /// Periodo para ejecución de rutinas con frecuencia 1Hz

/*******************************************************************************
 *           MANEJO DE PRESENCE VECTOR
 *******************************************************************************/

// Set/Report Wrench Effort
#define PRESENCE_VECTOR_THROTTLE 0x0001 /// Obtención de PV de acelerador en mensajes JAUS Set/Report Wrench Effort
#define PRESENCE_VECTOR_STEER 0x0010 /// Obtención de PV de dirección en mensajes JAUS Set/Report Wrench Effort
#define PRESENCE_VECTOR_BRAKE 0x0040 /// Obtención de PV de freno de servicio en mensajes JAUS Set/Report Wrench Effort

// Set/Report Discrete devices
#define PRESENCE_VECTOR_GEAR 0x04 /// Obtención de PV de marcha en mensajes JAUS Set/Report Discrete devices
#define PRESENCE_VECTOR_PARKING_BRAKE 0x02 /// Obtención de PV de freno de estacionamiento en mensajes JAUS Set/Report Discrete devices

// Report Camera Pose
#define PRESENCE_VECTOR_CURRENT_PAN 0x0040 /// Obtención de PV de PAN en mensajes JAUS Report Camera Pose
#define PRESENCE_VECTOR_CURRENT_TILT 0x0020 /// Obtención de PV de TILT en mensajes JAUS Report Camera Pose
#define PRESENCE_VECTOR_CURRENT_ZOOM 0x0008 /// Obtención de PV de ZOOM en mensajes JAUS Report Camera Pose 

// Set Camera Pose
#define PRESENCE_VECTOR_PAN 0x20 /// Obtención de PV de PAN en mensajes JAUS Set Camera Pose
#define PRESENCE_VECTOR_TILT 0x10 /// Obtención de PV de TILT en mensajes JAUS Set Camera Pose
#define PRESENCE_VECTOR_ZOOM 0x04 /// Obtención de PV de ZOOM en mensajes JAUS Set Camera Pose

// UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_LEVEL 0x01 /// Obtención de PV de nivel de batería en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_VOLTAGE 0x02 /// Obtención de PV de tensión de batería en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_CURRENT 0x04 /// Obtención de PV de intensidad de batería en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_TEMPERATURE 0x08 /// Obtención de PV de temperatura de batería en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_MOTOR_TEMPERATURE 0x10 /// Obtención de PV de temperatura de motor en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_MOTOR_RPM 0x20 /// Obtención de PV de rpm de motor en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_ALARMS 0x40  /// Obtención de PV de alarmas eléctricas en mensajes JAUS UGV Info Exp #12

// Report Velocity State
#define PRESENCE_VECTOR_CURRENT_CRUISING_SPEED 0x0008  /// Obtención de PV de velocidad de crucero en mensajes JAUS Report Velocity State

// Report Signaling Elements
#define PRESENCE_VECTOR_BLINKER_RIGHT 0x10 /// Obtención de PV de intermitente derecho en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_BLINKER_LEFT 0x80 /// Obtención de PV de intermitente izquierdo en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_KLAXON 0x20 /// Obtención de PV de bocina en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_DIPSP 0x02 /// Obtención de PV de luces de posición en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_DIPSR 0x04 /// Obtención de PV de luces largas en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_DIPSS 0x01 /// Obtención de PV de luces cortas en mensajes JAUS Report Signaling Elements

/*******************************************************************************
 ********************************************************************************
 *                       ARTEFACTOS JAUS
 ********************************************************************************
 *******************************************************************************/

/*******************************************************************************
 *                        SUBSISTEMAS JAUS
 *******************************************************************************/
#define JAUS_SUBSYSTEM_MYC 1 /// Identificador de subsistema JAUS C2
#define JAUS_SUBSYSTEM_USV 2 /// Identificador de subsistema JAUS USV
#define JAUS_SUBSYSTEM_UGV 3 /// Identificador de subsistema JAUS UGV

/*******************************************************************************
 *                           NODOS JAUS
 *******************************************************************************/
#define JAUS_NODE_CONTROL 1 /// Identificador de nodo JAUS Control
#define JAUS_NODE_TABLET 2 /// Identificador de nodo JAUS Tablet
#define JAUS_NODE_CAMERA 2 /// Identificador de nodo JAUS Camera
#define JAUS_NODE_COMM_MNG 3 /// Identificador de nodo JAUS Communication Mng
#define JAUS_NODE_AOC 4 /// Identificador de nodo JAUS AOC

/*******************************************************************************
 *              IDENTIFICADOR DE CAMARA DE APOYO A LA CONDUCCION
 ******************************************************************************/
#define FRONT_CAMERA_ID 1 /// Identificador de cámara de apoyo a la conducción delantera
#define REAR_CAMERA_ID 2 /// Identificador de cámara de apoyo a la conducción trasera

#endif	/* CONSTANT_H */

/**
 * @}
 */