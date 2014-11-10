/* 
 * File:   constant.h
 * Author: Carlos Amores
 *
 * Created on 4 de junio de 2014, 17:27
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif



/*******************************************************************************
 * PROTOCOLO PAYLOAD DE CONDUCCION             
 * IDENTIFICADOR DE INSTRUCCION
*******************************************************************************/

#define SET 0
#define GET 1
#define INFO 2
#define ACK 3
#define NACK 4

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
#define OPERATION_MODE_SWITCH 57



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
 *                   SOCKET PAYLOAD DE CONDUCCION
*******************************************************************************/

#define IP_PAYLOAD_CONDUCCION_DRIVING "127.0.0.1"
#define PORT_PAYLOAD_CONDUCCION_DRIVING 10000

/*******************************************************************************
 * FRECUENCIA DE REQUERIMIENTO DE INFORMACION A DISPOSITIVO
 ******************************************************************************/

#define FREC_10HZ 0.1
#define FREC_5HZ 0.2
#define FREC_2HZ 0.5
#define FREC_1HZ 1

/*******************************************************************************
 *           ESTRUCTURA DE INTERCAMBIO CON PAYLOAD DE CONDUCCION
*******************************************************************************/

typedef struct{
    short instruction;
    short id_instruction;
    short element;
    short value;
}FrameDriving;


/*******************************************************************************
 *           MÁSCARAS DE IDENTIFICACIÓN DE ALARMAS
 *******************************************************************************/

#define MASK_NOT_ALARMS 0x0000 ///<Indicador de todas las alarmas a 0
#define MASK_ALARMS_CONNECTION_STEERING_FAILED 0x0001 ///<Indicador de fallo de conexión
#define MASK_ALARMS_STEERING_FAILED 0x0002 ///<Indicador de fallo en la dirección
#define MASK_ALARMS_BRAKE_CONNECTION_FAILED 0x0004 ///<Indicador de fallo de conexión con freno de servicio
#define MASK_ALARMS_BRAKE_FAILED 0x0008 ///<Indicador de fallo de freno de servicio
#define MASK_ALARMS_HANDBRAKE_CONNECTION_FALED 0x0010 ///<Indicador de fallo de conexión con freno de estacionamiento
#define MASK_ALARMS_HANDBRAKE_FAILED 0x0020 ///<Indicador de fallo de freno de estacionamiento
#define MASK_ALARMS_MOTOR_TEMPERATURE 0x0040 ///<Indicador de alta temperatura de motor
#define MASK_ALARMS_FLAGS_FAILED 0x0080 ///< Indicador de fallo en los testigos
#define MASK_ALARMS_ACC_FAILED 0x0100 ///<Indicador de fallo en aceleración
#define MASK_ALARMS_GEAR_FAILED 0x0200 ///<Indicador de fallo en el cambio de marchas

#endif	/* CONSTANT_H */
