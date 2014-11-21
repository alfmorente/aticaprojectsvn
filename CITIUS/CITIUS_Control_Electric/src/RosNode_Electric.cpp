
/** 
 * @file  RosNode_Electric.cpp
 * @brief Implementacion de la clase "RosNode_Electric"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode_Electric.h"

/**
 * Constructor de la clase. Inicia la maquina de estados del nodo y crea la 
 * instancia del driver de conexión con el vehículo
 */
RosNode_Electric::RosNode_Electric() {
  nodeStatus = NODESTATUS_INIT;
  dElectric = new ElectricConnectionManager();
}

/**
 * Destructor de la clase
 */
RosNode_Electric::~RosNode_Electric() {
  delete(dElectric);
}

/**
 * Método privado que inicializa los artefactos ROS atributos de la clase. 
 * Consulta la posición del conmutador local/teleoperado y solicita la inicialización de la maquina de
 * estados de modos de operacion del vehículo con la señal obtenida
 */
void RosNode_Electric::initROS() {
  ros::NodeHandle nh;
  clientVehicleStatus = nh.serviceClient<CITIUS_Control_Electric::srv_vehicleStatus>("vehicleStatus");
  pubElectricInfo = nh.advertise<CITIUS_Control_Electric::msg_electricInfo>("electricInfo", 1000);
  pubCommand = nh.advertise<CITIUS_Control_Electric::msg_command>("command", 1000);
  pubSwitcher = nh.advertise<CITIUS_Control_Electric::msg_switcher>("switcher", 1000);
  subsElectricCommand = nh.subscribe("electricCommand", 1000, &RosNode_Electric::fcn_sub_electricCommand,this);
  ROS_INFO("[Control] Electric - Solicitando inicio de nodos del vehiculo");
  dElectric->setTurnOn();
  CITIUS_Control_Electric::srv_vehicleStatus service;
  service.request.status = OPERATION_MODE_INICIANDO;
  // Solicitar a vehículo posicion conmutador local/teleoperado
  ROS_INFO("[Control] Electric - Esperando posicion del conmutador local/teleoperado para arrancar");
  //service.request.posSwitcher = dElectric->waitForSwitcherPosition();
  // Cambiar (parche)!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  service.request.posSwitcher = SWITCHER_TELECONTROL;
  while (!clientVehicleStatus.call(service)) {
    ros::spinOnce();
  }
  if (service.response.confirmation) {
    ROS_INFO("[Control] Electric - Se ha iniciado el vehiculo");
    nodeStatus = NODESTATUS_OK;
  } else {
    ROS_INFO("[Control] Electric - El vehiculo no ha podido iniciar");
    nodeStatus = NODESTATUS_OFF;
  }
}

/**
 * Método privado consumidor del topic para la recepción de mensajes ROS con 
 * comandos de actuacion sobre el vehículo. Lo transmite el vehículo y lo encola 
 * si el elemento es considerado crítico y por tanto debe llevarse a cabo el 
 * mecanismo de integridad
 * @param[in] msg Mensaje ROS recibido
 */
void RosNode_Electric::fcn_sub_electricCommand(CITIUS_Control_Electric::msg_electricCommand msg){
  if(msg.id_device == SUPPLY_TURN_ON ||
          msg.id_device == SUPPLY_5 || 
          msg.id_device == SUPPLY_12 ||
          msg.id_device == SUPPLY_24_DRIVE ||
          msg.id_device == SUPPLY_24_OCC ||
          msg.id_device == SUPPLY_48 ||
          msg.id_device == CONTROL_SYSTEM_SUPPLY ||
          msg.id_device == DRIVE_SYSTEM_SUPPLY ||
          msg.id_device == COMM_SYSTEM_SUPPLY ||
          msg.id_device == OBSERVATION_SYSTEM_SUPPLY) {
    ROS_INFO("[Control] Driving - Comando de telecontrol recibido: %d:=%d", msg.id_device, msg.value);
    FrameDriving command;
    command.instruction = SET;
    command.id_instruction = -1;
    command.element = static_cast<DeviceID> (msg.id_device);
    command.value = msg.value;
    dElectric->sendToVehicle(command);
  } else {
    ROS_INFO("[Control] Electric - Descartado comando - Dispositivo incorrecto");
  }
}

/**
 * Método público consultor del atributo "dElectric" de la clase que 
 * proporciona la instancia del driver utilizado en la comunicación con el 
 * vehículo
 * @return Atributo "dElectric" de la clase
 */
ElectricConnectionManager *RosNode_Electric::getDriverMng() {
  return dElectric;
}

/**
 * Método público que publica la información del vehículo que recibe como 
 * parámetro en el topic ROS correspondiente
 * @param[in] info Información eléctrica del vehículo a publicar
 */
void RosNode_Electric::publishElectricInfo(ElectricInfo info) {
  CITIUS_Control_Electric::msg_electricInfo msg;
  msg.battery_level = info.battery_level;
  msg.battery_voltage = info.battery_voltage;
  msg.battery_current = info.battery_current;
  msg.battery_temperature = info.battery_temperature;
  msg.supply_alarms = info.supply_alarms;
  pubElectricInfo.publish(msg);
}

/**
 * Método público que publica la información de un cambio en la posición del 
 * conmutador local / teleoperado que recibe como parámetro en el topic ROS 
 * correspondiente
 * @param[in] position Nueva posición leida del conmutador local / teleoperado
 */
void RosNode_Electric::publishSwitcherInfo(short position) {
  CITIUS_Control_Electric::msg_switcher msg;
  msg.switcher = position;
  pubSwitcher.publish(msg);
}

/**
 * Método público que publica una serie de comandos de activacion sobre diversos 
 * actuadores ante una lectura de cambio en el conmutador local / teleoperado
 * @param[in] on Nueva posición del conmutador local / teleoperado
 */
void RosNode_Electric::publishSetupCommands(bool on) {
  CITIUS_Control_Electric::msg_command msg;
  if (on)
    msg.value = 1;
  else
    msg.value = 0;
  msg.id_device = MT_THROTTLE;
  pubCommand.publish(msg);
  msg.id_device = MT_BRAKE;
  pubCommand.publish(msg);
  msg.id_device = MT_HANDBRAKE;
  pubCommand.publish(msg);
  msg.id_device = MT_STEERING;
  pubCommand.publish(msg);
  msg.id_device = MT_GEAR;
  pubCommand.publish(msg);
  msg.id_device = MT_LIGHTS;
  pubCommand.publish(msg);
  msg.id_device = MT_BLINKERS;
  pubCommand.publish(msg);
}
/**
 * Método público que comprueba si el último mensaje recibido del vehículo es
 * una petición de finalizar la ejecución y da soporte en caso afirmativo
 */
void RosNode_Electric::checkTurnOff() {
  if (dElectric->getTurnOffFlag()) {
    CITIUS_Control_Electric::srv_vehicleStatus srvTurnOff;
    srvTurnOff.request.status = OPERATION_MODE_APAGANDO;
    while (!clientVehicleStatus.call(srvTurnOff)) {
      ros::spinOnce();
    }
    if (srvTurnOff.response.confirmation) {
      ros::NodeHandle nh;
      nh.setParam("vehicleStatus", OPERATION_MODE_APAGANDO);
      dElectric->setTurnOff();
      dElectric->disconnect();
      nodeStatus = NODESTATUS_OFF;
    } else {
      ROS_INFO("[Control] Electric - Sin confirmacion para apagar el vehiculo");
    }
  }
}

/**
 * Método público que comprueba si el último mensaje recibido del vehículo es 
 * un cambio de posición en el conmutador local / teleoperado y da soporte al
 * mismo en caso afirmativo
 */
void RosNode_Electric::checkSwitcher() {
  if (dElectric->getSwitcherStruct().flag) {
    publishSwitcherInfo(dElectric->getSwitcherStruct().position);
    if (dElectric->getSwitcherStruct().position == 1) {
      publishSetupCommands(true);
    } else if (dElectric->getSwitcherStruct().position == 0) {
      publishSetupCommands(false);
    }
    dElectric->setSwitcherStruct(false);
  }
}

/**
 * Método público que comprueba si se ha recibido un nuevo mensaje con el vector
 * de alarmas del vehículo y se encarga de su tratamiento
 */
void RosNode_Electric::checkSupplyAlarms() {
  if (dElectric->getSupplyAlarmsStruct().flag) {
    if ((dElectric->getSupplyAlarmsStruct().supplyAlarms | MASK_NOT_ALARMS) == MASK_NOT_ALARMS) {
      if (nodeStatus == NODESTATUS_CORRUPT) {
        ROS_INFO("[Control] Electric - Sin alarmas activas - Nodo en modo OK");
        nodeStatus = NODESTATUS_OK;
        CITIUS_Control_Electric::msg_command msg;
        msg.id_device = SUPPLY_ALARMS;
        msg.value = 0;
        pubCommand.publish(msg);
      }
    } else {
      if (nodeStatus == NODESTATUS_OK) {
        ROS_INFO("[Control] Electric - Alarmas activas - Nodo DEGRADADO");
        nodeStatus = NODESTATUS_CORRUPT;
        CITIUS_Control_Electric::msg_command msg;
        msg.id_device = SUPPLY_ALARMS;
        msg.value = 1;
        pubCommand.publish(msg);
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_BATTERY_TEMPERATURE) == MASK_ALARMS_BATTERY_TEMPERATURE) {
        ROS_INFO("[Control] Electric - Alarma: Temperatura de las baterias");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_BATTERY_VOLTAGE) == MASK_ALARMS_BATTERY_VOLTAGE) {
        ROS_INFO("[Control] Electric - Alarma: Tension de las baterias");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_BATTERY_CURRENT) == MASK_ALARMS_BATTERY_CURRENT) {
        ROS_INFO("[Control] Electric - Alarma: Corriente de las baterias");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_48V_FAILED) == MASK_ALARMS_48V_FAILED) {
        ROS_INFO("[Control] Electric - Alarma: Fallo en DC/DC 48V");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_24V_DRIVING_FAILED) == MASK_ALARMS_24V_DRIVING_FAILED) {
        ROS_INFO("[Control] Electric - Alarma: Fallo en DC/DC 24V (Conduccion)");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_24V_FAILED) == MASK_ALARMS_24V_FAILED) {
        ROS_INFO("[Control] Electric - Alarma: Fallo en DC/DC 24V (Resto)");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_12V_FAILED) == MASK_ALARMS_12V_FAILED) {
        ROS_INFO("[Control] Electric - Alarma: Fallo en DC/DC 12V");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_5V_FAILED) == MASK_ALARMS_5V_FAILED) {
        ROS_INFO("[Control] Electric - Alarma: Fallo en DC/DC 5V");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_CONTROL_FAILED) == MASK_ALARMS_CONTROL_FAILED) {
        ROS_INFO("[Control] Electric - Alarma: Fallo en Control");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_DRIVING_FAILED) == MASK_ALARMS_DRIVING_FAILED) {
        ROS_INFO("[Control] Electric - Alarma: Fallo en Payload de conduccion");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_COMM_FAILED) == MASK_ALARMS_COMM_FAILED) {
        ROS_INFO("[Control] Electric - Alarma: Fallo en Comunicaciones");
      }
      if ((dElectric->getSupplyAlarmsStruct().supplyAlarms & MASK_ALARMS_OBSERVATION_FAILED) == MASK_ALARMS_OBSERVATION_FAILED) {
        ROS_INFO("[Control] Electric - Alarma: Fallo en Payload de observacion");
      }

    }
    dElectric->setSupplyAlarmsStruct(false);
  }
}