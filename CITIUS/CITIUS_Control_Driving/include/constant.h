
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de la comunicacion 
 * entre el nodo Driving del Subsistema de control y el modulo de conduccion
 * del Subsitema de payload de conduccion
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

using namespace std;

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
#define SUPPLY_ALARMS 56.
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

typedef struct {
  short instruction; /// Tipo de instruccion
  short id_instruccion; /// Identificador para mecanismo de integridad
  short element; /// Elemento de demanda
  short value; /// Valor de demanda
} FrameDriving;

/*******************************************************************************
 *           ESTRUCTURA DE INFORMACION DE VEHICULO
 *******************************************************************************/

typedef struct {
  short steering; /// Valor de direccion
  short thottle; /// Valor de acelerador
  short brake; /// Valor de freno de servicio
  bool parkingBrake; /// Valor de freno de estacionamiento
  unsigned short gear; /// Valor de marcha
  unsigned short speed; /// Valor de velocidad de crucero
  short motorRPM; /// Valor de r.p.m. de motor
  short motorTemperature; /// Valor de temperatura de motor
  bool lights; /// Indicador de informacion de luces en la estructura
  bool blinkerLeft; /// Valor de intermitente izquierdo
  bool blinkerRight; /// Valor de iontermitente derecho
  bool dipss; /// Valor de luces cortas
  bool dipsr; /// Valor de luces largas
  bool dipsp; /// Valor de luces de posicion
  bool klaxon; /// Valor de vocina
} DrivingInfo;

/*******************************************************************************
 *           ESTRUCTURA DE MANEJO DE NACK's
 *******************************************************************************/

typedef struct {
  int numOfMsgs; /// Numero de mensajes a retransmitir
  vector<FrameDriving> msgs; /// Coleccion de mensajes a retransmitir
} RtxStruct;

#endif	/* CONSTANT_H */



