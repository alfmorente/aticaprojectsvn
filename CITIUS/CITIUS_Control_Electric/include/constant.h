
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de la comunicación 
 * entre el nodo Electric del Subsistema de control y el módulo eléctrico del
 * Subsitema de payload de conducción
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup ElectricDriver
 * @{
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

#include <vector>

using namespace std;

/*******************************************************************************
 * PROTOCOLO PAYLOAD DE CONDUCCION             
 * IDENTIFICADOR DE INSTRUCCION
 *******************************************************************************/

/**
 * \enum CommandID
 * \brief Identificador de comando para protocolo con Payload de conducción
 */
typedef enum {
  SET = 0, ///<Valor para generación de instrucción SET
  GET = 1, ///<Valor para generación de instrucción GET
  INFO = 2, ///<Valor para generación de instrucción INFO
  ACK = 3, ///<Valor para generación de instrucción ACK
  NACK = 4 ///<Valor para generación de instrucción NACK
} CommandID;

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
 * FICHERO DE CONFIGURACION
 ******************************************************************************/

#define CONFIG_FILE_IP_NAME "IP" ///<Identificador de búsqueda en fichero de IP
#define CONFIG_FILE_PORT_NAME "PORT" ///<Identificador de búsqueda en fichero de PORT

/*******************************************************************************
 *                              ESTADOS DEL NODO
 *******************************************************************************/

/**
 * \enum NodeStatus
 * \brief Tipos para estado local de nodo
 */
typedef enum {
  NODESTATUS_INIT = 0, ///<Identificador del estado INIT de la máquina de estados de nodo
  NODESTATUS_OK = 1, ///<Identificador del estado OK de la máquina de estados de nodo
  NODESTATUS_CORRUPT = 2, ///<Identificador del estado CORRUPT de la máquina de estados de nodo
  NODESTATUS_OFF = 3 ///<Identificador del estado OFF de la máquina de estados de nodo
} NodeStatus;

/*******************************************************************************
 *                          MODOS DE OPERACION DEL VEHICULO
 *******************************************************************************/

/**
 * \enum OperationMode
 * \brief Tipos para modos de operación
 */
typedef enum {
  OPERATION_MODE_LOCAL = 0, ///<Identificador del modo de operación LOCAL de la máquina de estados del vehículo
  OPERATION_MODE_INICIANDO = 1, ///<Identificador del modo de operación OPERATION_MODE_INICIANDO de la máquina de estados del vehículo
  OPERATION_MODE_CONDUCCION = 2, ///<Identificador del modo de operación OPERATION_MODE_CONDUCCION de la máquina de estados del vehículo
  OPERATION_MODE_OBSERVACION = 3, ///<Identificador del modo de operación OPERATION_MODE_OBSERVACION de la máquina de estados del vehículo
  OPERATION_MODE_APAGANDO = 4 ///<Identificador del modo de operación OPERATION_MODE_APAGANDO de la máquina de estados del vehículo        
} OperationMode;

/*******************************************************************************
 * FRECUENCIA -> PERIODO
 ******************************************************************************/

#define FREC_10HZ 0.1 ///<Periodo para ejecución de rutinas con frecuencia 10Hz
#define FREC_5HZ 0.2 ///<Periodo para ejecución de rutinas con frecuencia 5Hz
#define FREC_2HZ 0.5 ///<Periodo para ejecución de rutinas con frecuencia 2Hz
#define FREC_1HZ 1 ///<Periodo para ejecución de rutinas con frecuencia 1Hz

/*******************************************************************************
 * IDENTIFICADORES DE DISPOSITIVO
 ******************************************************************************/

#define DEVICE_XSENS 1 ///<Identificador para abrir el fichero de configuración del socket XSENS
#define DEVICE_AHRS 2 ///<Identificador para abrir el fichero de configuración del socket AHRS
#define DEVICE_DRIVING 3 ///<Identificador para abrir el fichero de configuración del socket DRIVING
#define DEVICE_ELECTRIC 4 ///<Identificador para abrir el fichero de configuración del socket ELECTRIC

/*******************************************************************************
 *           ESTRUCTURA DE INTERCAMBIO CON PAYLOAD DE CONDUCCION
 *******************************************************************************/

/**
 * \struct FrameDriving
 * \brief Estructura de intercambio con payload de conducción
 */
typedef struct {
  CommandID instruction; ///<Tipo de instruccion
  short id_instruction; ///<Identificador para mecanismo de integridad
  DeviceID element; ///<Elemento de demanda
  short value; ///<Valor de demanda
} FrameDriving;

/*******************************************************************************
 *           MAXIMO NUMERO DE INTENTOS DE CONEXION
 *******************************************************************************/

#define MAX_ATTEMPS 5 ///<Náximo número de reintentos de conexión

/*******************************************************************************
 *              POSICION CONMUTADOR LOCAL - TELEOPERADOR
 *******************************************************************************/
#define SWITCHER_INIT -1 ///<Valor para posición del conmutador previa lectura
#define SWITCHER_LOCAL 0 ///<Valor para posición del conmutador LOCAL
#define SWITCHER_TELECONTROL 1 ///<Valor para posición del conmutador TELEOPERADO

/*******************************************************************************
 *           ESTRUCTURA DE INFORMACION ELECTRICA DE VEHICULO
 *******************************************************************************/

/**
 * \struct ElectricInfo
 * \brief Estructura contenedor de información del vehículo
 */
typedef struct {
  short battery_level; ///<Valor del nivel de bateria
  short battery_voltage; ///<Valor de voltaje de bateria
  short battery_current; ///<Valor de intensidad de bateria
  short battery_temperature; ///<Valor de temperatura de bateria
  short supply_alarms; ///<Valor del vector de alarmas del subsistema
} ElectricInfo;

/**
 * \struct SystemSupplies
 * \brief Estructura contenedor de los sistemas de alimentación de los distintos
 * módulos
 */
typedef struct{
  short controlSystemSupply;
  short controlSystemSupply5;
  short controlSystemSupply12;
  short controlSystemSupply24;
  short controlSystemSupply48;
  short driveSystemSupply;
  short driveSystemSupply5;
  short driveSystemSupply12;
  short driveSystemSupply24;
  short driveSystemSupply48;
  short commSystemSupply;
  short commSystemSupply5;
  short commSystemSupply12;
  short commSystemSupply24;
  short commSystemSupply48;
  short observationSystemSupply;
  short observationSystemSupply5;
  short observationSystemSupply12;
  short observationSystemSupply24;
  short observationSystemSupply48;
} SystemSupplies;

/*******************************************************************************
 *           ESTRUCTURA DE MANEJO DE NACK's
 *******************************************************************************/

/**
 * \struct RtxStruct
 * \brief Estructura para manejo de retransmisión de mensajes ante recepción de
 * NACK
 */
typedef struct {
  int numOfMsgs; ///<Numero de mensajes a retransmitir
  vector<FrameDriving> msgs; ///<Coleccion de mensajes a retransmitir
} RtxStruct;

/*******************************************************************************
 *           ESTRUCTURA DE POSICION DEL CONMUTADOR LOCAL/TELEOPERADO
 *******************************************************************************/

/**
 * \struct SwitcherStruct
 * \brief Estructura para manejo de indicaciones de cambio en la posición del
 * conmutador Local/Teleoperado
 */
typedef struct {
  bool flag; ///<Indicador de cambio en posicion del conmutador
  short position; ///<Posicion del conmutador tras el cambio
} SwitcherStruct;

/*******************************************************************************
 *           ESTRUCTURA DE VECTOR DE ALARMAS
 *******************************************************************************/

/**
 * \struct SupplyAlarmsStruct
 * \brief Estructura para manejo de recepción y tratamiento del vector de
 * alarmas del módulo eléctrico del Payload de conducción
 */
typedef struct {
  bool flag; ///<Indicador de cambio en el vector de alarmas
  short supplyAlarms; ///<Vector de alarmas
} SupplyAlarmsStruct;

/*******************************************************************************
 *           MÁSCARAS DE IDENTIFICACIÓN DE ALARMAS
 *******************************************************************************/

#define MASK_NOT_ALARMS 0x0000 ///<Indicador de todas las alarmas a 0
#define MASK_ALARMS_BATTERY_TEMPERATURE 0x0001 ///<Indicador de temperatura de las baterías
#define MASK_ALARMS_BATTERY_VOLTAGE 0x0002 ///<Indicador de tensión de las baterías
#define MASK_ALARMS_BATTERY_CURRENT 0x0004 ///<Indicador de corriente de las baterías 
#define MASK_ALARMS_48V_FAILED 0x0008 ///<Indicador de fallo en DC/DC 48V
#define MASK_ALARMS_24V_DRIVING_FAILED 0x0010 ///<Indicador de fallo en DC/DC 24V (conducción)
#define MASK_ALARMS_24V_FAILED 0x0020 ///<Indicador de fallo en DC/DC 24V (resto)
#define MASK_ALARMS_12V_FAILED 0x0040 ///<Indicador de fallo en DC/DC 12V
#define MASK_ALARMS_5V_FAILED 0x0080 ///<Indicador de fallo en DC/DC 5V
#define MASK_ALARMS_CONTROL_FAILED 0x0100 ///<Indicador de fallo subsistema de control
#define MASK_ALARMS_DRIVING_FAILED 0x0200 ///<Indicador de fallo en subsistema de Payload de conducción
#define MASK_ALARMS_COMM_FAILED 0x0400 ///<Indicador de fallo en subsistema de comunicaciones
#define MASK_ALARMS_OBSERVATION_FAILED 0x0800 ///<Indicador de fallo en subsistema de Payload de observación

#endif	/* CONSTANT_H */

/**
 * @}
 */