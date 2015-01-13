
/** 
 * @file  constant.h
 * @brief Declara las constantes necesarias para el manejo de mensajes JAUS y
 * ROS en interior y exterior de la arquitectura.
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup JAUSModule
 * @{
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

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

/**
 * \enum JausOperationMode
 * \brief Tipos para modos de operación (JAUS)
 */
typedef enum {
  JAUS_OPERATION_MODE_INICIANDO = 0, ///<Identificador del modo de operación INICIANDO de la máquina de estados del vehículo para transmitir en mensaje JAUS
  JAUS_OPERATION_MODE_LOCAL = 1, ///<Identificador del modo de operación LOCAL de la máquina de estados del vehículo para transmitir en mensaje JAUS
  JAUS_OPERATION_MODE_CONDUCCION = 5, ///<Identificador del modo de operación CONDUCCION de la máquina de estados del vehículo para transmitir en mensaje JAUS
  JAUS_OPERATION_MODE_OBSERVACION = 6 ///<Identificador del modo de operación OBSERVACION de la máquina de estados del vehículo para transmitir en mensaje JAUS
} JausOperationMode;

#define FREC_10HZ 0.1 ///<Periodo para ejecución de rutinas con frecuencia 10Hz
#define FREC_5HZ 0.2 ///<Periodo para ejecución de rutinas con frecuencia 5Hz
#define FREC_2HZ 0.5 ///<Periodo para ejecución de rutinas con frecuencia 2Hz
#define FREC_1HZ 1 ///<Periodo para ejecución de rutinas con frecuencia 1Hz

// Set/Report Wrench Effort
#define PRESENCE_VECTOR_THROTTLE 0x0001 ///<Obtención de PV de acelerador en mensajes JAUS Set/Report Wrench Effort
#define PRESENCE_VECTOR_STEER 0x0020 ///<Obtención de PV de dirección en mensajes JAUS Set/Report Wrench Effort
#define PRESENCE_VECTOR_BRAKE 0x0040 ///<Obtención de PV de freno de servicio en mensajes JAUS Set/Report Wrench Effort

// Set/Report Discrete devices
#define PRESENCE_VECTOR_GEAR 0x04 ///<Obtención de PV de marcha en mensajes JAUS Set/Report Discrete devices
#define PRESENCE_VECTOR_PARKING_BRAKE 0x02 ///<Obtención de PV de freno de estacionamiento en mensajes JAUS Set/Report Discrete devices

// Report Camera Pose
#define PRESENCE_VECTOR_CURRENT_PAN 0x0040 ///<Obtención de PV de PAN en mensajes JAUS Report Camera Pose
#define PRESENCE_VECTOR_CURRENT_TILT 0x0020 ///<Obtención de PV de TILT en mensajes JAUS Report Camera Pose
#define PRESENCE_VECTOR_CURRENT_ZOOM 0x0008 ///<Obtención de PV de ZOOM en mensajes JAUS Report Camera Pose 

// Set Camera Pose
#define PRESENCE_VECTOR_PAN 0x20 ///<Obtención de PV de PAN en mensajes JAUS Set Camera Pose
#define PRESENCE_VECTOR_TILT 0x10 ///<Obtención de PV de TILT en mensajes JAUS Set Camera Pose
#define PRESENCE_VECTOR_ZOOM 0x04 ///<Obtención de PV de ZOOM en mensajes JAUS Set Camera Pose

// UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_LEVEL 0x01 ///<Obtención de PV de nivel de batería en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_VOLTAGE 0x02 ///<Obtención de PV de tensión de batería en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_CURRENT 0x04 ///<Obtención de PV de intensidad de batería en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_BATTERY_TEMPERATURE 0x08 ///<Obtención de PV de temperatura de batería en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_MOTOR_TEMPERATURE 0x10 ///<Obtención de PV de temperatura de motor en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_MOTOR_RPM 0x20 ///<Obtención de PV de rpm de motor en mensajes JAUS UGV Info Exp #12
#define PRESENCE_VECTOR_ALARMS 0x40  ///<Obtención de PV de alarmas eléctricas en mensajes JAUS UGV Info Exp #12

// Report Velocity State
#define PRESENCE_VECTOR_CURRENT_CRUISING_SPEED 0x0008  ///<Obtención de PV de velocidad de crucero en mensajes JAUS Report Velocity State

// Report Signaling Elements
#define PRESENCE_VECTOR_BLINKER_RIGHT 0x10 ///<Obtención de PV de intermitente derecho en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_BLINKER_LEFT 0x80 ///<Obtención de PV de intermitente izquierdo en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_KLAXON 0x20 ///<Obtención de PV de bocina en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_DIPSP 0x02 ///<Obtención de PV de luces de posición en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_DIPSR 0x04 ///<Obtención de PV de luces largas en mensajes JAUS Report Signaling Elements
#define PRESENCE_VECTOR_DIPSS 0x01 ///<Obtención de PV de luces cortas en mensajes JAUS Report Signaling Elements

// Conversion % a mensajes JAUS (Report Camera Pose)
#define CONV_JAUS_PANTILT 1/100 ///<Factor de conversión de % (camara) a 0..1 (Pan y Tilt en Report Camera Pose)
#define CONV_JAUS_ZOOM 32/100 ///<Factor de conversión de % (camara) a 0..32 (Zoom en Report Camera Pose)

// Identificadores de alarmas a enviar hacia MyC
#define ID_ALARMS_NOT_ALARMS 0 ///<Identificador de fin de alarmas en Control
#define ID_ALARMS_WRONG_TURN_OFF 1 ///<Identificador de alarma de apagado incorrecto previo
#define ID_ALARMS_DRIVING 2 ///<Identificador de alarma producida en módulo de conducción
#define ID_ALARMS_ELECTRIC 3 ///<Identificador de alarma producida en módulo eléctrico
#define ID_ALARMS_DRIVING_ELECTRIC 4 ///<Identificador de alarmas en módulos eléctrico y conducción

/**
 * \enum JausSubsystemID
 * \brief Identificador de subsistemas JAUS
 */
typedef enum {
  JAUS_SUBSYSTEM_MYC = 1, ///<Identificador de subsistema JAUS C2
  JAUS_SUBSYSTEM_USV = 2, ///<Identificador de subsistema JAUS USV
  JAUS_SUBSYSTEM_UGV = 3 ///<Identificador de subsistema JAUS UGV
} JausSubsystemID;

/**
 * \enum JausNodeID
 * \brief Identificador de subsistemas JAUS
 */
typedef enum {
  JAUS_NODE_CONTROL = 3, ///<Identificador de nodo JAUS Control
  JAUS_NODE_TABLET = 2, ///<Identificador de nodo JAUS Tablet
  JAUS_NODE_CAMERA = 2, ///<Identificador de nodo JAUS Camera
  JAUS_NODE_COMM_MNG = 1, ///<Identificador de nodo JAUS Communication Mng
  JAUS_NODE_AOC = 4 ///<Identificador de nodo JAUS AOC
} JausNodeID;

#define JAUS_DESTINANTION_INSTANCE 1 ///<Instancia JAUS destino 

/**
 * \enum CameraID
 * \brief Identificador de cámaras
 */
typedef enum {
  FRONT_CAMERA_ID = 1, ///<Identificador de cámara de apoyo a la conducción delantera
  REAR_CAMERA_ID = 2, ///<Identificador de cámara de apoyo a la conducción trasera
  TV_CAMERA = 3, ///<Identificador de cámara diurna Payload de Observación
  IR_CAMERA = 4 ///<Identificador de cámara nocturna Payload de Observación
} CameraID;

/**
 * \enum HeartBeatPosition
 * \brief Estructura con información para formar mensaje de HeartBeat de
 * posicionamiento del vehículo. Se actualiza con la información recibida de 
 * los dispositivos de posición/orientación.
 */
typedef struct{
  double latitude;
  double longitude;
  double altitude;
  double heading;
  double speed;
}HeartBeatPosition;


#endif	/* CONSTANT_H */

/**
 * @}
 */