/* 
 * File:   RosNode_Joystick.h
 * Author: atica
 *
 * Created on 2 de marzo de 2015, 11:10
 */

#ifndef ROSNODE_JOYSTICK_H
#define	ROSNODE_JOYSTICK_H

#include "sensor_msgs/Joy.h"
//#include "RcvJoy/msg_command.h"
#include "../../Common_files/msg_gen/cpp/include/Common_files/msg_com_teleop.h"


#include "ros/ros.h"


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

#define ACC_MAX 100
#define ACC_MAX_XBOX 50
// Botones
/*#define BTN_LUCES_POSICION 0
#define BTN_LUCES_CORTAS 1
#define BTN_LUCES_LARGAS 2
#define BTN_CLAXON 3
#define BTN_BAJAR_MARCHA 4
#define BTN_SUBIR_MARCHA 5
#define BTN_HIDRAULICO 8
#define BTN_ARRANQUE 9
#define BTN_FRENOMANO 11

// Ejes

#define EJE_DIRECCION 0
#define EJE_ACELERADOR 2
#define EJE_INTERMITENTE 5*/

// XBOX
// Botones
#define BTN_LUCES_POSICION 2
#define BTN_LUCES_CORTAS 0
#define BTN_LUCES_LARGAS 1
#define BTN_CLAXON 3
#define BTN_BAJAR_MARCHA 4
#define BTN_SUBIR_MARCHA 5
#define BTN_HIDRAULICO 6
#define BTN_ARRANQUE 7
#define BTN_FRENOMANO 10

// Ejes

#define EJE_DIRECCION 0
#define EJE_ACELERADOR 5
#define EJE_INTERMITENTE 6
#define EJE_FRENO 2

typedef struct{
  int valor;
  bool changing;
} EstadoSimple;

typedef struct{
  int valor;
  bool changingUp;
  bool changingDown;
} EstadoComplejo;


typedef struct{
  EstadoSimple luces_cortas;
  EstadoSimple luces_largas;
  EstadoSimple luces_posicion;
  EstadoSimple claxon;
  int direccion;
  int aceleracion;
  int freno_servicio;
  EstadoSimple freno_mano;
  EstadoComplejo marcha;
  EstadoSimple arranque;
  EstadoSimple hidraulico;
  EstadoSimple int_derecho;
  EstadoSimple int_izquierdo;
} EstadoComandos;

class RosNode_Joystick{
public:
  RosNode_Joystick();
  ~RosNode_Joystick();
  void initROS();
private:
  EstadoComandos estados;
  ros::Subscriber subsJoystick;
  ros::Publisher pubCommand;
  void fcn_sub_joy(sensor_msgs::Joy msg);
};

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
  //OPERATION_MODE_SWITCH = 58 ///<Valor para formación de comando OPERATION_MODE_SWITCH
  ARRANQUE = 58,
  NIVELCOMBUSTIBLE = 59,
  RELEHIDRAULICO = 60,
  DIPSP = 61,
  DIPSS = 62,
  DIPSR = 63,
  MANUALAUTOMATICO = 64
} DeviceID;


#endif	/* ROSNODE_JOYSTICK_H */

