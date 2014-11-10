
/** 
 * @file  RosNode_Driving.cpp
 * @brief Implementación de la clase "RosNode_Driving"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode_Driving.h"

/**
 * Constructor de la clase. Inicia la máquina de estados del nodo y crea la 
 * instancia del driver de conexión con el vehículo
 */
RosNode_Driving::RosNode_Driving() {
  nodeStatus = NODESTATUS_INIT;
  dVehicle = new DrivingConnectionManager();
  electricAlarms = false;
}

/**
 * Destructor de la clase
 */
RosNode_Driving::~RosNode_Driving() {
  delete(dVehicle);
}

/**
 * Método privado que inicia los artefactos ROS atributos de la clase
 */
void RosNode_Driving::initROS() {
  ros::NodeHandle nh;
  pubVehicleInfo = nh.advertise<CITIUS_Control_Driving::msg_vehicleInfo>("vehicleInfo", 1000);
  subsCommand = nh.subscribe("command", 1000, &RosNode_Driving::fcn_sub_command, this);
  servNodeStatus = nh.advertiseService("vmNodeStatus", &RosNode_Driving::fcv_serv_nodeStatus, this);
}

/**
 * Método privado consumidor del topic para la recepción de mensajes ROS con 
 * comandos de actuacion sobre el vehículo. Lo transmite el vehículo y lo encola 
 * si el elemento es considerado crítico y por tanto debe llevarse a cabo el 
 * mecanismo de integridad
 * @param[in] msg Mensaje ROS recibido
 */
void RosNode_Driving::fcn_sub_command(CITIUS_Control_Driving::msg_command msg) {
  // Comprobacion de aviso de alarmas en Electric
  if (msg.id_device == SUPPLY_ALARMS) {
    // Alarmas modulo electrico: OFF
    if (msg.value == 0) {
      electricAlarms = false;
      dVehicle->setAlarmsStruct(true);
    }// Alarmas modulo electrico: ON
    else {
      electricAlarms = true;
      dVehicle->setAlarmsStruct(true);
    }
    return;
  }
  ros::NodeHandle nh;
  int currentVehicleStatus = OPERATION_MODE_INICIANDO;
  nh.getParam("vehicleStatus", currentVehicleStatus);
  if (currentVehicleStatus == OPERATION_MODE_CONDUCCION || dVehicle->isMTCommand(static_cast<DeviceID> (msg.id_device))) {
    if (nodeStatus == NODESTATUS_OK || dVehicle->isMTCommand(static_cast<DeviceID> (msg.id_device))) {
      ROS_INFO("[Control] Driving - Comando de telecontrol recibido: %d:=%d", msg.id_device, msg.value);
      if (checkCommand(msg)) {
        // Envio de comando a vehículo
        FrameDriving command;
        command.instruction = SET;
        command.element = static_cast<DeviceID> (msg.id_device);
        command.value = msg.value;
        if (dVehicle->isCriticalInstruction(static_cast<DeviceID> (msg.id_device))) {
          short cont = dVehicle->getCountCriticalMessages();
          // Valor de ID_INSTRUCCION
          command.id_instruccion = cont;
          // Introduccion en la cola de mensajes críticos
          dVehicle->addToQueue(command);
          // Incremento del contador
          dVehicle->setCountCriticalMessages(cont + 1);
        } else {
          command.id_instruccion = -1;
        }
        dVehicle->sendToVehicle(command);
      } else {
        ROS_INFO("[Control] Driving - Descartado comando %d:=%d - Fuera de rango", msg.id_device, msg.value);
      }
    } else {
      ROS_INFO("[Control] Driving - Descartado comando %d:=%d - Nodo en estado %d", msg.id_device, msg.value, nodeStatus);
    }
  } else {
    ROS_INFO("[Control] Driving - Descartado comando %d:=%d - Vehiculo fuera del modo CONDUCCION", msg.id_device, msg.value);
  }
}

/**
 * Método privado encargado del tratamiento de servicios para la modificación de
 * la máquina de estados del nodo
 * @param[in] rq Parámetros de requerimiento
 * @param[in] rsp Parámetros de respuesta
 * @return Booleano que indica si se ha realizado el correcto tratamiento de
 * la petición  de servicio
 */
bool RosNode_Driving::fcv_serv_nodeStatus(CITIUS_Control_Driving::srv_nodeStatus::Request &rq, CITIUS_Control_Driving::srv_nodeStatus::Response &rsp) {
  if (rq.status == NODESTATUS_OK) {
    nodeStatus = NODESTATUS_OK;
    rsp.confirmation = true;
  } else if (rq.status == NODESTATUS_OFF) {
    dVehicle->disconnect();
    nodeStatus = NODESTATUS_OFF;
    rsp.confirmation = true;
  } else {
    rsp.confirmation = false;
  }
  return true;
}

/**
 * Método público consultor del atributo "dVehicle" de la clase que proporciona 
 * la instancia  del driver utilizado en la comunicación con el vehículo
 * @return Atributo "dVehicle" de la clase
 */
DrivingConnectionManager *RosNode_Driving::getDriverMng() {
  return dVehicle;
}

/**
 * Método privado que realiza la criba de comandos recibidos si el valor a 
 * imprimir sobre un elemento esta fuera de los límites establecidos para ese 
 * elemento
 * @param[in] msg Mensaje ROS a comprobar
 * @return Booleano que indica si el comando es válido (valor dentro de los 
 * límites establecidos para dicho elemento) o no
 */
bool RosNode_Driving::checkCommand(CITIUS_Control_Driving::msg_command msg) {
  bool ret = true;
  short value = msg.value;
  switch (msg.id_device) {
    case (RESET):
      if (value < 0 || value > 1) ret = false;
      break;
    case BLINKER_RIGHT:
      if (value < 0 || value > 1) ret = false;
      break;
    case BLINKER_LEFT:
      if (value < 0 || value > 1) ret = false;
      break;
    case BLINKER_EMERGENCY:
      if (value < 0 || value > 1) ret = false;
      break;
    case MT_BLINKERS:
      if (value < 0 || value > 1) ret = false;
      break;
    case DIPSP:
      if (value < 0 || value > 1) ret = false;
      break;
    case DIPSS:
      if (value < 0 || value > 1) ret = false;
      break;
    case DIPSR:
      if (value < 0 || value > 1) ret = false;
      break;
    case KLAXON:
      if (value < 0 || value > 1) ret = false;
      break;
    case MT_LIGHTS:
      if (value < 0 || value > 1) ret = false;
      break;
    case GEAR:
      if (value < 0 || value > 2) ret = false;
      break;
    case MT_GEAR:
      if (value < 0 || value > 1) ret = false;
      break;
    case THROTTLE:
      if (value < 0 || value > 100) ret = false;
      break;
    case CRUISING_SPEED:
      if (value < 0 || value > 1000) ret = false;
      break;
    case MT_THROTTLE:
      if (value < 0 || value > 1) ret = false;
      break;
    case HANDBRAKE:
      if (value < 0 || value > 1) ret = false;
      break;
    case MT_HANDBRAKE:
      if (value < 0 || value > 1) ret = false;
      break;
    case BRAKE:
      if (value < 0 || value > 100) ret = false;
      break;
    case MT_BRAKE:
      if (value < 0 || value > 1) ret = false;
      break;
    case STEERING:
      if (value < -100 || value > 100) ret = false;
      break;
    case MT_STEERING:
      if (value < 0 || value > 1) ret = false;
      break;
    default:
      break;
  };
  return ret;
}

/**
 * Método público que publica la información del vehículo que recibe como 
 * parámetro en el topic ROS correspondiente
 * @param[in] info Información del vehículo a publicar
 */
void RosNode_Driving::publishDrivingInfo(DrivingInfo info) {
  CITIUS_Control_Driving::msg_vehicleInfo msg;

  msg.steering = info.steering;
  msg.thottle = info.thottle;
  msg.brake = info.brake;
  msg.parkingBrake = info.parkingBrake;
  msg.gear = info.gear;
  msg.speed = info.speed;
  msg.motorRPM = info.motorRPM;
  msg.motorTemperature = info.motorTemperature;
  msg.lights = info.lights;
  msg.blinkerLeft = info.blinkerLeft;
  msg.blinkerRight = info.blinkerRight;
  msg.dipss = info.dipss;
  msg.dipsr = info.dipsr;
  msg.dipsp = info.dipsp;
  msg.klaxon = info.klaxon;
  pubVehicleInfo.publish(msg);
}

/**
 * Método público que comprueba si ha habido cambio en cualquiera de los
 * vectores de alarmas del sistema y da tratamiento en caso afirmativo
 */
void RosNode_Driving::checkAlarms() {
  if (dVehicle->getAlarmsStruct().flag) {
    if (((dVehicle->getAlarmsStruct().driveAlarms | MASK_NOT_ALARMS) == MASK_NOT_ALARMS) && !electricAlarms) {
      if (nodeStatus == NODESTATUS_CORRUPT) { // Fin de las alarmas en ambos subsistemas
        ROS_INFO("[Control] Driving - Nodo en estado OK - Sin alarmas");
        nodeStatus = NODESTATUS_OK;
      }
    } else if (((dVehicle->getAlarmsStruct().driveAlarms | MASK_NOT_ALARMS) == MASK_NOT_ALARMS) && electricAlarms) {
      if (nodeStatus == NODESTATUS_OK) { // Nuevas alarmas en Electric
        ROS_INFO("[Control] Driving - Nodo en estado DEGRADADO - Alarmas en Electric");
        // Rutina de emergencia - Parada de vehiculo
        setEmergecyCommands();
        nodeStatus = NODESTATUS_CORRUPT;
      } else { // Deja de haber alarmas en Driving
        ROS_INFO("[Control] Driving - Modo en estado DEGRADADO - Fin de alarmas en Driving pero hay alarmas en Electric");
      }
    } else if (((dVehicle->getAlarmsStruct().driveAlarms | MASK_NOT_ALARMS) != MASK_NOT_ALARMS) && !electricAlarms) {
      if (nodeStatus == NODESTATUS_OK) { // Nuevas alarmas en Driving
        ROS_INFO("[Control] Driving - Nodo en estado DEGRADADO - Alarmas en Driving");
        // Rutina de emergencia - Parada de vehiculo
        setEmergecyCommands();
        nodeStatus = NODESTATUS_CORRUPT;
        // Identificacion de alarms
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_CONNECTION_STEERING_FAILED) == MASK_ALARMS_CONNECTION_STEERING_FAILED) {
          ROS_INFO("[Control] Driving - Alarma: Fallo de conexion");
        }
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_STEERING_FAILED) == MASK_ALARMS_STEERING_FAILED) {
          ROS_INFO("[Control] Driving - Alarma: Fallo en sistema de direccion");
        }
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_BRAKE_CONNECTION_FAILED) == MASK_ALARMS_BRAKE_CONNECTION_FAILED) {
          ROS_INFO("[Control] Driving - Alarma: Fallo de conexion con freno de servicio");
        }
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_BRAKE_FAILED) == MASK_ALARMS_BRAKE_FAILED) {
          ROS_INFO("[Control] Driving - Alarma: Fallo en freno de servicio");
        }
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_HANDBRAKE_CONNECTION_FALED) == MASK_ALARMS_HANDBRAKE_CONNECTION_FALED) {
          ROS_INFO("[Control] Driving - Alarma: Fallo de conexion con freno de estacionamiento");
        }
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_HANDBRAKE_FAILED) == MASK_ALARMS_HANDBRAKE_FAILED) {
          ROS_INFO("[Control] Driving - Alarma: Fallo en freno de estacionamiento");
        }
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_MOTOR_TEMPERATURE) == MASK_ALARMS_MOTOR_TEMPERATURE) {
          ROS_INFO("[Control] Driving - Alarma: Temperatura de motor");
        }
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_FLAGS_FAILED) == MASK_ALARMS_FLAGS_FAILED) {
          ROS_INFO("[Control] Driving - Alarma: Fallo en testigos");
        }
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_ACC_FAILED) == MASK_ALARMS_ACC_FAILED) {
          ROS_INFO("[Control] Driving - Alarma: Fallo en aceleracion");
        }
        if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_GEAR_FAILED) == MASK_ALARMS_GEAR_FAILED) {
          ROS_INFO("[Control] Driving - Alarma: Fallo en cambio de marchas");
        }
      } else { // Deja de haber alarmas en Electric
        ROS_INFO("[Control] Driving - Modo en estado DEGRADADO - Fin de alarmas en Electric pero hay alarmas en Driving");
      }
    } else if (((dVehicle->getAlarmsStruct().driveAlarms | MASK_NOT_ALARMS) != MASK_NOT_ALARMS) && electricAlarms) {
      ROS_INFO("[Control] Driving - Modo en estado DEGRADADO - Alarmas en Driving & Electric");
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_CONNECTION_STEERING_FAILED) == MASK_ALARMS_CONNECTION_STEERING_FAILED) {
        ROS_INFO("[Control] Driving - Alarma: Fallo de conexion");
      }
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_STEERING_FAILED) == MASK_ALARMS_STEERING_FAILED) {
        ROS_INFO("[Control] Driving - Alarma: Fallo en sistema de direccion");
      }
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_BRAKE_CONNECTION_FAILED) == MASK_ALARMS_BRAKE_CONNECTION_FAILED) {
        ROS_INFO("[Control] Driving - Alarma: Fallo de conexion con freno de servicio");
      }
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_BRAKE_FAILED) == MASK_ALARMS_BRAKE_FAILED) {
        ROS_INFO("[Control] Driving - Alarma: Fallo en freno de servicio");
      }
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_HANDBRAKE_CONNECTION_FALED) == MASK_ALARMS_HANDBRAKE_CONNECTION_FALED) {
        ROS_INFO("[Control] Driving - Alarma: Fallo de conexion con freno de estacionamiento");
      }
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_HANDBRAKE_FAILED) == MASK_ALARMS_HANDBRAKE_FAILED) {
        ROS_INFO("[Control] Driving - Alarma: Fallo en freno de estacionamiento");
      }
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_MOTOR_TEMPERATURE) == MASK_ALARMS_MOTOR_TEMPERATURE) {
        ROS_INFO("[Control] Driving - Alarma: Temperatura de motor");
      }
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_FLAGS_FAILED) == MASK_ALARMS_FLAGS_FAILED) {
        ROS_INFO("[Control] Driving - Alarma: Fallo en testigos");
      }
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_ACC_FAILED) == MASK_ALARMS_ACC_FAILED) {
        ROS_INFO("[Control] Driving - Alarma: Fallo en aceleracion");
      }
      if ((dVehicle->getAlarmsStruct().driveAlarms & MASK_ALARMS_GEAR_FAILED) == MASK_ALARMS_GEAR_FAILED) {
        ROS_INFO("[Control] Driving - Alarma: Fallo en cambio de marchas");
      }
    }

    // Clear del indicador de cambio
    dVehicle->setAlarmsStruct(false);
  }
}

/**
 * Método privado que envía una serie de comandos al vehículo para deternerlo 
 * ante una emergencia
 */
void RosNode_Driving::setEmergecyCommands() {
  ROS_INFO("[Control] Driving - Rutina de seguridad - Detencion del vehiculo");
  FrameDriving command;
  command.instruction = SET;

  // Acelerador a 0
  command.element = THROTTLE;
  command.value = 0;
  short cont = dVehicle->getCountCriticalMessages();
  // Valor de ID_INSTRUCCION
  command.id_instruccion = cont;
  // Introduccion en la cola de mensajes críticos
  dVehicle->addToQueue(command);
  // Incremento del contador
  dVehicle->setCountCriticalMessages(cont + 1);
  // Envio a dispositivo
  dVehicle->sendToVehicle(command);

  // Direccion a 0
  command.element = STEERING;
  cont = dVehicle->getCountCriticalMessages();
  // Valor de ID_INSTRUCCION
  command.id_instruccion = cont;
  // Introduccion en la cola de mensajes críticos
  dVehicle->addToQueue(command);
  // Incremento del contador
  dVehicle->setCountCriticalMessages(cont + 1);
  // Envio a dispositivo
  dVehicle->sendToVehicle(command);

  // Marcha a 0
  command.element = GEAR;
  cont = dVehicle->getCountCriticalMessages();
  // Valor de ID_INSTRUCCION
  command.id_instruccion = cont;
  // Introduccion en la cola de mensajes críticos
  dVehicle->addToQueue(command);
  // Incremento del contador
  dVehicle->setCountCriticalMessages(cont + 1);
  // Envio a dispositivo
  dVehicle->sendToVehicle(command);

  // Freno a 100
  command.element = BRAKE;
  command.value = 100;
  cont = dVehicle->getCountCriticalMessages();
  // Valor de ID_INSTRUCCION
  command.id_instruccion = cont;
  // Introduccion en la cola de mensajes críticos
  dVehicle->addToQueue(command);
  // Incremento del contador
  dVehicle->setCountCriticalMessages(cont + 1);
  // Envio a dispositivo
  dVehicle->sendToVehicle(command);

  // Freno de estacionamiento a 1
  command.element = HANDBRAKE;
  command.value = 1;
  cont = dVehicle->getCountCriticalMessages();
  // Valor de ID_INSTRUCCION
  command.id_instruccion = cont;
  // Introduccion en la cola de mensajes críticos
  dVehicle->addToQueue(command);
  // Incremento del contador
  dVehicle->setCountCriticalMessages(cont + 1);
  // Envio a dispositivo
  dVehicle->sendToVehicle(command);

}