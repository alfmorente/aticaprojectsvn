
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de la comunicación 
 * entre el nodo Electric del Subsistema de control y el módulo eléctrico del
 * Subsitema de payload de conducción
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

using namespace std;

/*******************************************************************************
 * PROTOCOLO PAYLOAD DE CONDUCCION             
 * IDENTIFICADOR DE INSTRUCCION
 *******************************************************************************/

#define SET 0 /// Valor para generación de instrucción SET
#define GET 1 /// Valor para generación de instrucción GET
#define INFO 2 /// Valor para generación de instrucción INFO
#define ACK 3 /// Valor para generación de instrucción ACK
#define NACK 4 /// Valor para generación de instrucción NACK

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
 *                   SOCKET PAYLOAD DE CONDUCCION
 *******************************************************************************/

#define IP_PAYLOAD_CONDUCCION_DRIVING "127.0.0.1" /// Dirección IP para creación del socket de comunicación
#define PORT_PAYLOAD_CONDUCCION_DRIVING 10000 /// Puerto para creación del socket de comunicación

/*******************************************************************************
 * FRECUENCIA -> PERIODO
 ******************************************************************************/

#define FREC_10HZ 0.1 /// Periodo para ejecución de rutinas con frecuencia 10Hz
#define FREC_5HZ 0.2 /// Periodo para ejecución de rutinas con frecuencia 5Hz
#define FREC_2HZ 0.5 /// Periodo para ejecución de rutinas con frecuencia 2Hz
#define FREC_1HZ 1 /// Periodo para ejecución de rutinas con frecuencia 1Hz

/*******************************************************************************
 *           ESTRUCTURA DE INTERCAMBIO CON PAYLOAD DE CONDUCCION
 *******************************************************************************/

/**
 * /struct FrameDriving
 * /brief Estructura de intercambio con payload de conducción
 */
typedef struct {
  short instruction; /// Tipo de instruccion
  short id_instruction; /// Identificador para mecanismo de integridad
  short element; /// Elemento de demanda
  short value; /// Valor de demanda
} FrameDriving;

/*******************************************************************************
 *           MAXIMO NUMERO DE INTENTOS DE CONEXION
 *******************************************************************************/

#define MAX_ATTEMPS 5 /// Náximo número de reintentos de conexión

/*******************************************************************************
 *              POSICION CONMUTADOR LOCAL - TELEOPERADOR
 *******************************************************************************/
#define SWITCHER_INIT -1 /// Valor para posición del conmutador previa lectura
#define SWITCHER_LOCAL 0 /// Valor para posición del conmutador LOCAL
#define SWITCHER_TELECONTROL 1 /// Valor para posición del conmutador TELEOPERADO

/*******************************************************************************
 *           ESTRUCTURA DE INFORMACION ELECTRICA DE VEHICULO
 *******************************************************************************/

/**
 * /struct ElectricInfo
 * /brief Estructura contenedor de información del vehículo
 */
typedef struct {
  short battery_level; /// Valor del nivel de bateria
  short battery_voltage; /// Valor de voltaje de bateria
  short battery_current; /// Valor de intensidad de bateria
  short battery_temperature; /// Valor de temperatura de bateria
  short supply_alarms; /// Valor del vector de alarmas del subsistema
} ElectricInfo;

/*******************************************************************************
 *           ESTRUCTURA DE MANEJO DE NACK's
 *******************************************************************************/

/**
 * /struct RtxStruct
 * /brief Estructura para manejo de retransmisión de mensajes ante recepción de
 * NACK
 */
typedef struct {
  int numOfMsgs; /// Numero de mensajes a retransmitir
  vector<FrameDriving> msgs; /// Coleccion de mensajes a retransmitir
} RtxStruct;

/*******************************************************************************
 *           ESTRUCTURA DE POSICION DEL CONMUTADOR LOCAL/TELEOPERADO
 *******************************************************************************/

/**
 * /struct SwitcherStruct
 * /brief Estructura para manejo de indicaciones de cambio en la posición del
 * conmutador Local/Teleoperado
 */
typedef struct {
  bool flag; /// Indicador de cambio en posicion del conmutador
  short position; /// Posicion del conmutador tras el cambio
} SwitcherStruct;

#endif	/* CONSTANT_H */

/**
 * @}
 */