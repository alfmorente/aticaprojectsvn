/* 
 * File:   constantes.h
 * Author: atica
 *
 * Created on 19 de febrero de 2015, 12:39
 */

#ifndef CONSTANTES_H
#define	CONSTANTES_H

#include <vector>

using namespace std;

/**
 * \enum CommandID
 * \brief Identificador de comando para protocolo con Payload de conducción
 */
typedef enum {
  A_SET = 0, ///<Valor para generación de instrucción SET
  GET = 1, ///<Valor para generación de instrucción GET
  A_INFO = 2, ///<Valor para generación de instrucción INFO
  ACK = 3, ///<Valor para generación de instrucción ACK
  NACK = 4 ///<Valor para generación de instrucción NACK
} CommandID;

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
  //DIPSP = 5, ///<Valor para formación de comando DIPSP
  //DIPSS = 6, ///<Valor para formación de comando DIPSS
  //DIPSR = 7, ///<Valor para formación de comando DIPSR
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
  ABRAKE = 19, ///<Valor para formación de comando BRAKE
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
  //OPERATION_MODE_SWITCH = 58, ///<Valor para formación de comando OPERATION_MODE_SWITCH  
  ARRANQUE = 58,
  NIVELCOMBUSTIBLE = 59,
  RELEHIDRAULICO = 60,
  DIPSP = 61,
  DIPSS = 62,
  DIPSR = 63,
  MANUALAUTOMATICO = 64,
  ESTADO_MOTOR = 65          
} DeviceID;

#define CONFIG_FILE_IP_NAME "IP" ///<Identificador de búsqueda en fichero de IP
#define CONFIG_FILE_PORT_NAME "PORT" ///<Identificador de búsqueda en fichero de PORT

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

#define FREC_10HZ 0.1 ///<Periodo para ejecución de rutinas con frecuencia 10Hz
#define FREC_5HZ 0.2 ///<Periodo para ejecución de rutinas con frecuencia 5Hz
#define FREC_2HZ 0.5 ///<Periodo para ejecución de rutinas con frecuencia 2Hz
#define FREC_1HZ 1 ///<Periodo para ejecución de rutinas con frecuencia 1Hz

#define DEVICE_XSENS 1 ///<Identificador para abrir el fichero de configuración del socket XSENS
#define DEVICE_AHRS 2 ///<Identificador para abrir el fichero de configuración del socket AHRS
#define DEVICE_DRIVING 3 ///<Identificador para abrir el fichero de configuración del socket DRIVING
#define DEVICE_ELECTRIC 4 ///<Identificador para abrir el fichero de configuración del socket ELECTRIC

/**
 * \struct FrameDriving
 * \brief Estructura de intercambio con payload de conducción
 */
typedef struct {
  CommandID instruction; ///<Tipo de instruccion
  short id_instruccion; ///<Identificador para mecanismo de integridad
  DeviceID element; ///<Elemento de demanda
  short value; ///<Valor de demanda
} FrameDriving;

/**
 * \struct DrivingInfo
 * \brief Estructura contenedor de información del vehículo
 */
typedef struct {
  short steering; ///<Valor de direccion
  short thottle; ///<Valor de acelerador
  short brake; ///<Valor de freno de servicio
  bool parkingBrake; ///<Valor de freno de estacionamiento
  unsigned short gear; ///<Valor de marcha
  unsigned short speed; ///<Valor de velocidad de crucero
  short motorRPM; ///<Valor de r.p.m. de motor
  short motorTemperature; ///<Valor de temperatura de motor
  bool lights; ///<Indicador de informacion de luces en la estructura
  bool blinkerLeft; ///<Valor de intermitente izquierdo
  bool blinkerRight; ///<Valor de intermitente derecho
  bool dipss; ///<Valor de luces cortas
  bool dipsr; ///<Valor de luces largas
  bool dipsp; ///<Valor de luces de posicion
  bool klaxon; ///<Valor de vocina
  bool rengine;
  short alarms; ///<Valor de alarmas
} DrivingInfo;

/**
 * \struct RtxStruct
 * \brief Estructura para manejo de retransmisión de mensajes ante recepción de
 * NACK
 */
typedef struct {
  int numOfMsgs; ///<Numero de mensajes a retransmitir
  vector<FrameDriving> msgs; ///<Coleccion de mensajes a retransmitir
} RtxStruct;

/**
 * \struct AlarmsStruct
 * \brief Estructura para manejo de recepción y tratamiento del vector de
 * alarmas del módulo de conducción del Payload de conducción
 */
typedef struct {
  bool flag; ///<Indicador de cambio en recepción del vector de alarmas
  short driveAlarms; ///<Vector de alarmas del vehículo
  short steeringAlarms; ///<Vector de alarmas del sistema de dirección
} AlarmsStruct;

#define MASK_NOT_ALARMS 0x0000 ///<Indicador de todas las alarmas a 0
#define MASK_ALARMS_CONNECTION_STEERING_FAILED 0x0001 ///<Indicador de fallo de conexión con sistema de dirección
#define MASK_ALARMS_STEERING_FAILED 0x0002 ///<Indicador de fallo en la dirección
#define MASK_ALARMS_BRAKE_CONNECTION_FAILED 0x0004 ///<Indicador de fallo de conexión con freno de servicio
#define MASK_ALARMS_BRAKE_FAILED 0x0008 ///<Indicador de fallo de freno de servicio
#define MASK_ALARMS_HANDBRAKE_CONNECTION_FALED 0x0010 ///<Indicador de fallo de conexión con freno de estacionamiento
#define MASK_ALARMS_HANDBRAKE_FAILED 0x0020 ///<Indicador de fallo de freno de estacionamiento
#define MASK_ALARMS_MOTOR_TEMPERATURE 0x0040 ///<Indicador de alta temperatura de motor
#define MASK_ALARMS_FLAGS_FAILED 0x0080 ///< Indicador de fallo en los testigos
#define MASK_ALARMS_ACC_FAILED 0x0100 ///<Indicador de fallo en aceleración
#define MASK_ALARMS_GEAR_FAILED 0x0200 ///<Indicador de fallo en el cambio de marchas

/**
 * \struct SwitcherStruct
 * \brief Estructura para manejo de indicaciones de cambio en la posición del
 * conmutador Local/Teleoperado
 */
typedef struct {
  bool flag; ///<Indicador de cambio en posicion del conmutador
  short position; ///<Posicion del conmutador tras el cambio
} SwitcherStruct;

#define SWITCHER_INIT -1 ///<Valor para posición del conmutador previa lectura
#define SWITCHER_LOCAL 0 ///<Valor para posición del conmutador LOCAL
#define SWITCHER_TELECONTROL 1 ///<Valor para posición del conmutador TELEOPERADO

/**
 * \struct UpdateReg
 * \brief Estructura para manejo de la entrada en la cola de mensajes críticos 
 * de un actuador específico
 */
typedef struct {
  bool flag; ///<Indicador de comando esperando para enviar
  short value; ///<Valor del comando 
} UpdateReg;

/**
 * \struct UpdateReg
 * \brief Estructura contenedora de todos los registros de actualización que 
 * permiten la entrada en la cola de mensajes críticos
 */
typedef struct {
  UpdateReg throttle; ///<Registro de actualización para acelerador
  UpdateReg brake; ///<Registro de actualización para freno de servicio
  UpdateReg handBrake; ///<Registro de actualización para freno de mano
  UpdateReg gear; ///<Registro de actualización para marcha
  UpdateReg speed; ///<Registro de actualización para velocidad de crucero
  UpdateReg steering; ///<Registro de actualización para dirección
} UpdateRegs;

// Identificadores de alarmas a enviar hacia MyC
#define ID_ALARMS_NOT_ALARMS 0 ///<Identificador de fin de alarmas en Control
#define ID_ALARMS_WRONG_TURN_OFF 1 ///<Identificador de alarma de apagado incorrecto previo
#define ID_ALARMS_DRIVING 2 ///<Identificador de alarma producida en módulo de conducción
#define ID_ALARMS_ELECTRIC 3 ///<Identificador de alarma producida en módulo eléctrico
#define ID_ALARMS_DRIVING_ELECTRIC 4 ///<Identificador de alarmas en módulos eléctrico y conducción


#endif	/* CONSTANTES_H */

