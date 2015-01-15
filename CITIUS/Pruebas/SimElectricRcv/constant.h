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

#endif	/* CONSTANT_H */


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

/**
 * \enum DeviceID
 * \brief Identificador de dispositivo para protocolo con Payload de conducción
 */
typedef enum {
  RESET = 0, ///<Valor para formación de comando RESET
  BLINKER_RIGHT = 1, ///<Valor para formación de comando BLINKER_RIGHT
  BLINKER_LEFT = 2, ///<Valor para formación de comando BLINKER_LEFT
  BLINKER_EMERGENCY = 3, ///<Valor para formación de comando BLINKER_EMERGENCY
  MT_BLINKERS = 4, ///<Valor para formación de comando MT_BLINKERS
  DIPSP = 5, ///<Valor para formación de comando DIPSP
  DIPSS = 6, ///<Valor para formación de comando DIPSS
  DIPSR = 7, ///<Valor para formación de comando DIPSR
  KLAXON = 8, ///<Valor para formación de comando KLAXON
  MT_LIGHTS = 9, ///<Valor para formación de comando MT_LIGHTS
  GEAR = 10, ///<Valor para formación de comando GEAR
  MT_GEAR = 11, ///<Valor para formación de comando MT_GEAR
  THROTTLE = 12, ///<Valor para formación de comando THROTTLE
  MOTOR_RPM = 13, ///<Valor para formación de comando MOTOR_RPM
  CRUISING_SPEED = 14, ///<Valor para formación de comando CRUISING_SPEED
  MT_THROTTLE = 15, ///<Valor para formación de comando MT_THROTTLE
  MOTOR_TEMPERATURE = 16, ///<Valor para formación de comando MOTOR_TEMPERATURE
  HANDBRAKE = 17, ///<Valor para formación de comando HANDBRAKE
  MT_HANDBRAKE = 18, ///<Valor para formación de comando MT_HANDBRAKE
  BRAKE = 19, ///<Valor para formación de comando BRAKE
  MT_BRAKE = 20, ///<Valor para formación de comando MT_BRAKE
  STEERING = 21, ///<Valor para formación de comando STEERING
  STEERING_ALARMS = 22, ///<Valor para formación de comando STEERING_ALARMS
  MT_STEERING = 23, ///<Valor para formación de comando MT_STEERING
  DRIVE_ALARMS = 24, ///<Valor para formación de comando DRIVE_ALARMS
  BATTERY_LEVEL = 25, ///<Valor para formación de comando BATTERY_LEVEL
  BATTERY_VOLTAGE = 26, ///<Valor para formación de comando BATTERY_VOLTAGE
  BATTERY_CURRENT = 27, ///<Valor para formación de comando BATTERY_CURRENT
  BATTERY_TEMPERATURE = 28, ///<Valor para formación de comando BATTERY_TEMPERATURE
  SUPPLY_TURN_ON = 29, ///<Valor para formación de comando SUPPLY_TURN_ON
  SUPPLY_CHECK = 30, ///<Valor para formación de comando SUPPLY_CHECK
  TURN_OFF = 31, ///<Valor para formación de comando TURN_OFF
  SUPPLY_5 = 32, ///<Valor para formación de comando SUPPLY_5
  SUPPLY_12 = 33, ///<Valor para formación de comando SUPPLY_12
  SUPPLY_24_DRIVE = 34, ///<Valor para formación de comando SUPPLY_24_DRIVE
  SUPPLY_24_OCC = 35, ///<Valor para formación de comando SUPPLY_24_OCC
  SUPPLY_48 = 36, ///<Valor para formación de comando SUPPLY_48
  CONTROL_SYSTEM_SUPPLY = 37, ///<Valor para formación de comando CONTROL_SYSTEM_SUPPLY
  CONTROL_SYSTEM_SUPPLY_5 = 38, ///<Valor para formación de comando CONTROL_SYSTEM_SUPPLY_5
  CONTROL_SYSTEM_SUPPLY_12 = 39, ///<Valor para formación de comando CONTROL_SYSTEM_SUPPLY_12
  CONTROL_SYSTEM_SUPPLY_24 = 40, ///<Valor para formación de comando CONTROL_SYSTEM_SUPPLY_24
  CONTROL_SYSTEM_SUPPLY_48 = 41, ///<Valor para formación de comando CONTROL_SYSTEM_SUPPLY_48
  DRIVE_SYSTEM_SUPPLY = 42, ///<Valor para formación de comando DRIVE_SYSTEM_SUPPLY
  DRIVE_SYSTEM_SUPPLY_5 = 43, ///<Valor para formación de comando DRIVE_SYSTEM_SUPPLY_5
  DRIVE_SYSTEM_SUPPLY_12 = 44, ///<Valor para formación de comando DRIVE_SYSTEM_SUPPLY_12
  DRIVE_SYSTEM_SUPPLY_24 = 45, ///<Valor para formación de comando DRIVE_SYSTEM_SUPPLY_24
  DRIVE_SYSTEM_SUPPLY_48 = 46, ///<Valor para formación de comando DRIVE_SYSTEM_SUPPLY_48
  COMM_SYSTEM_SUPPLY = 47, ///<Valor para formación de comando COMM_SYSTEM_SUPPLY
  COMM_SYSTEM_SUPPLY_5 = 48, ///<Valor para formación de comando COMM_SYSTEM_SUPPLY_5
  COMM_SYSTEM_SUPPLY_12 = 49, ///<Valor para formación de comando COMM_SYSTEM_SUPPLY_12
  COMM_SYSTEM_SUPPLY_24 = 50, ///<Valor para formación de comando COMM_SYSTEM_SUPPLY_24
  COMM_SYSTEM_SUPPLY_48 = 51, ///<Valor para formación de comando COMM_SYSTEM_SUPPLY_48
  OBSERVATION_SYSTEM_SUPPLY = 52, ///<Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY
  OBSERVATION_SYSTEM_SUPPLY_5 = 53, ///<Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY_5
  OBSERVATION_SYSTEM_SUPPLY_12 = 54, ///<Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY_12
  OBSERVATION_SYSTEM_SUPPLY_24 = 55, ///<Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY_24
  OBSERVATION_SYSTEM_SUPPLY_48 = 56, ///<Valor para formación de comando OBSERVATION_SYSTEM_SUPPLY_48
  SUPPLY_ALARMS = 57, ///<Valor para formación de comando SUPPLY_ALARMS
  OPERATION_MODE_SWITCH = 58 ///<Valor para formación de comando OPERATION_MODE_SWITCH
} DeviceID;



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
#define PORT_PAYLOAD_CONDUCCION_DRIVING 10001

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

