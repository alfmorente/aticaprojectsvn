
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
  pubVehicleInfo = nh.advertise<Driving_Bobcat::msg_vehicleInfo>("vehicleInfo", 1000);
  subsCommand = nh.subscribe("command", 1000, &RosNode_Driving::fcn_sub_command, this);
  pubSwitcher = nh.advertise<Driving_Bobcat::msg_switcher>("switcher", 1000);
  servNodeStatus = nh.advertiseService("drNodeStatus", &RosNode_Driving::fcv_serv_nodeStatus, this);
  clientStatus = nh.serviceClient<Driving_Bobcat::srv_vehicleStatus>("vehicleStatus");
  ROS_INFO("[Control] Driving - Nodo listo para operar");
  nodeStatus = NODESTATUS_OK;
  nh.setParam("vehicleStatus",OPERATION_MODE_CONDUCCION);
}

/**
 * Método privado consumidor del topic para la recepción de mensajes ROS con 
 * comandos de actuacion sobre el vehículo. Lo transmite el vehículo y lo encola 
 * si el elemento es considerado crítico y por tanto debe llevarse a cabo el 
 * mecanismo de integridad
 * @param[in] msg Mensaje ROS recibido
 */
void RosNode_Driving::fcn_sub_command(Driving_Bobcat::msg_command msg) {
  if (msg.id_device == SUPPLY_ALARMS) {
    if (msg.value == 0) { // Todas OFF
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
          FrameDriving command;
          command.instruction = SET;
          command.element = static_cast<DeviceID> (msg.id_device);
          command.value = msg.value;
          dVehicle->setCommand(command);
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
bool RosNode_Driving::fcv_serv_nodeStatus(Driving_Bobcat::srv_nodeStatus::Request &rq, Driving_Bobcat::srv_nodeStatus::Response &rsp) {
  if (rq.status == NODESTATUS_OK) {
    nodeStatus = NODESTATUS_OK;
    rsp.confirmation = true;
  } else if (rq.status == NODESTATUS_OFF) {
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
bool RosNode_Driving::checkCommand(Driving_Bobcat::msg_command msg) {
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
  Driving_Bobcat::msg_vehicleInfo msg;
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
  msg.alarms = info.alarms;
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
        dVehicle->setAlarmsInfo(ID_ALARMS_NOT_ALARMS);
        nodeStatus = NODESTATUS_OK;
      }
    } else if (((dVehicle->getAlarmsStruct().driveAlarms | MASK_NOT_ALARMS) == MASK_NOT_ALARMS) && electricAlarms) {
      if (nodeStatus == NODESTATUS_OK) { // Nuevas alarmas en Electric
        ROS_INFO("[Control] Driving - Nodo en estado DEGRADADO - Alarmas en Electric");
        dVehicle->setAlarmsInfo(ID_ALARMS_ELECTRIC);
        setEmergecyCommands();
        nodeStatus = NODESTATUS_CORRUPT;
      } else { 
        ROS_INFO("[Control] Driving - Modo en estado DEGRADADO - Fin de alarmas en Driving pero hay alarmas en Electric");
        dVehicle->setAlarmsInfo(ID_ALARMS_DRIVING);
      }
    } else if (((dVehicle->getAlarmsStruct().driveAlarms | MASK_NOT_ALARMS) != MASK_NOT_ALARMS) && !electricAlarms) {
      if (nodeStatus == NODESTATUS_OK) { // Nuevas alarmas en Driving
        ROS_INFO("[Control] Driving - Nodo en estado DEGRADADO - Alarmas en Driving");
        dVehicle->setAlarmsInfo(ID_ALARMS_DRIVING);
        setEmergecyCommands();
        nodeStatus = NODESTATUS_CORRUPT;
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
        dVehicle->setAlarmsInfo(ID_ALARMS_DRIVING);
      }
    } else if (((dVehicle->getAlarmsStruct().driveAlarms | MASK_NOT_ALARMS) != MASK_NOT_ALARMS) && electricAlarms) {
      ROS_INFO("[Control] Driving - Modo en estado DEGRADADO - Alarmas en Driving & Electric");
      dVehicle->setAlarmsInfo(ID_ALARMS_DRIVING_ELECTRIC);
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
  // Acelerador
  command.element = THROTTLE;
  command.value = 0;
  dVehicle->setCommand(command);
  // Direccion
  command.element = STEERING;
  dVehicle->setCommand(command);
  // Marcha
  command.element = GEAR;
  dVehicle->setCommand(command);
  // Freno
  command.element = BRAKE;
  command.value = 100;
  dVehicle->setCommand(command);
  // Freno de estacionamiento
  command.element = HANDBRAKE;
  command.value = 1;
  dVehicle->setCommand(command);
}

/**
 * Método público que comprueba si el último mensaje recibido del vehículo es 
 * un cambio de posición en el conmutador local / teleoperado y da soporte al
 * mismo en caso afirmativo
 */
void RosNode_Driving::checkSwitcher() {
  if (dVehicle->getSwitcherStruct().flag) {
    publishSwitcherInfo(dVehicle->getSwitcherStruct().position);
    dVehicle->setSwitcherStruct(false);
  }
}


/**
 * Método público que publica la información de un cambio en la posición del 
 * conmutador local / teleoperado que recibe como parámetro en el topic ROS 
 * correspondiente
 * @param[in] position Nueva posición leida del conmutador local / teleoperado
 */
void RosNode_Driving::publishSwitcherInfo(short position) {
  Driving_Bobcat::msg_switcher msg;
  msg.switcher = position;
  pubSwitcher.publish(msg);
}
/**
 * Método privado que descarta un comando si la orden a transmitir se encuentra
 * actualmente activa en el vehículo
 * @param[in] element Identificador del actuador al que realizar el check
 * @param[in] value Valor para el actuador
 * @return Booleano que indica si el comando supera el corte o no
 */
bool RosNode_Driving::minimumRequiredInterval(short element, short value) {
  switch (element) {
    case BLINKER_RIGHT:
      return dVehicle->getLastCommandsInfo().blinkerRight != (bool)value;
      break;
    case BLINKER_LEFT:
      return dVehicle->getLastCommandsInfo().blinkerLeft != (bool)value;
      break;
    case DIPSP:
      return dVehicle->getLastCommandsInfo().dipsp != (bool)value;
      break;
    case DIPSS:
      return dVehicle->getLastCommandsInfo().dipss != (bool)value;
      break;
    case DIPSR:
      return dVehicle->getLastCommandsInfo().dipsr != (bool)value;
      break;
    case KLAXON:
      return dVehicle->getLastCommandsInfo().klaxon != (bool)value;
      break;
    case GEAR:
      return dVehicle->getLastCommandsInfo().gear != value;
      break;
    case THROTTLE:
      return (abs(dVehicle->getLastCommandsInfo().thottle - value) >= PERCENT_INTERVAL(MAX_INTERVAL_100,MIN_INTERVAL_100));
      break;
    case CRUISING_SPEED:
      return (abs(dVehicle->getLastCommandsInfo().speed - value) >= PERCENT_INTERVAL(MAX_INTERVAL_1000,MIN_INTERVAL_1000));
      break;
    case HANDBRAKE:
      return dVehicle->getLastCommandsInfo().parkingBrake != (bool)value;
      break;
    case BRAKE:
      return (abs(dVehicle->getLastCommandsInfo().brake - value) >= PERCENT_INTERVAL(MAX_INTERVAL_100,MIN_INTERVAL_100));
      break;
    case STEERING:
      return (abs(dVehicle->getLastCommandsInfo().steering - value) >= PERCENT_INTERVAL(MAX_INTERVAL_200,MIN_INTERVAL_200));
      break;
    default:
      return true;
      break;
  };
}
