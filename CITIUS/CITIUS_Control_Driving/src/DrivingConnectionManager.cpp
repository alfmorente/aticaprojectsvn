
/** 
 * @file  DrivingConnectionManager.cpp
 * @brief Implementacion de la clase "DrivingConnectionManager"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "DrivingConnectionManager.h"

/** 
 * Constructor de la clase
 */
DrivingConnectionManager::DrivingConnectionManager() {
  socketDescriptor = -1;
  countMsg = 1;
  vehicleInfo.lights = false;
  vehicleInfo.blinkerLeft = false;
  vehicleInfo.blinkerRight = false;
  vehicleInfo.dipsp = false;
  vehicleInfo.dipsr = false;
  vehicleInfo.dipss = false;
  vehicleInfo.klaxon = false;
  vehicleInfo.brake = 0;
  vehicleInfo.thottle = 0;
  vehicleInfo.steering = 0;
  vehicleInfo.parkingBrake = false;
  vehicleInfo.gear = 0;
  vehicleInfo.speed = 0;
  vehicleInfo.motorTemperature = 0;
  vehicleInfo.motorRPM = 0;
  driveAlarms = 0x0000;
  steeringAlarms = 0x0000;
}

/**
 * Destructor de la clase
 */
DrivingConnectionManager::~DrivingConnectionManager() {

}

/**
 * Método público que envía la información de una trama al vehículo haciendo uso 
 * del socket de comunicación
 * @param[in] frame Trama a enviar via socket al vehículo 
 */
void DrivingConnectionManager::sendToVehicle(FrameDriving frame) {

  // Buffer de envio
  char bufData[8];

  // AQUI FALLA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  // Rellenado del buffer
  memcpy(&bufData[0], &frame.instruction, sizeof (frame.instruction));
  memcpy(&bufData[2], &frame.id_instruccion, sizeof (frame.id_instruccion));
  memcpy(&bufData[4], &frame.element, sizeof (frame.element));
  memcpy(&bufData[6], &frame.value, sizeof (frame.value));

  // Envio via socket
  send(socketDescriptor, bufData, sizeof (bufData), 0);
  usleep(1000);
}

/**
 * Método público que envía una serie de tramas de tipo GET solicitando la 
 * información del vehículo
 * @param[in] full Indica si la solicitud a realizar es completa (true - 
 * actuadores y señalización) o parcial (false - actuadores únicamente)
 */
void DrivingConnectionManager::reqVehicleInfo(bool full) {
  FrameDriving frame;

  // Comun 
  frame.instruction = GET;
  frame.id_instruccion = -1;
  frame.value = -1;

  // Acelerador
  frame.element = THROTTLE;
  sendToVehicle(frame);
  // Freno 
  frame.element = BRAKE;
  sendToVehicle(frame);
  // Freno de mano
  frame.element = HANDBRAKE;
  sendToVehicle(frame);
  // Direccion
  frame.element = STEERING;
  sendToVehicle(frame);
  // Marcha
  frame.element = GEAR;
  sendToVehicle(frame);
  // Velocidad de crucero
  frame.element = CRUISING_SPEED;
  sendToVehicle(frame);
  // Temperatura de motor
  frame.element = MOTOR_TEMPERATURE;
  sendToVehicle(frame);
  // rpm de motor
  frame.element = MOTOR_RPM;
  sendToVehicle(frame);

  // Completa la peticion con elementos de señalizacion

  if (full) {
    // Luces de posicion
    frame.element = DIPSP;
    sendToVehicle(frame);
    // Luces cortas
    frame.element = DIPSS;
    sendToVehicle(frame);
    // Luces largas
    frame.element = DIPSR;
    sendToVehicle(frame);
    // Intermitente derecha
    frame.element = BLINKER_RIGHT;
    sendToVehicle(frame);
    // Intermitente izquierda
    frame.element = BLINKER_LEFT;
    sendToVehicle(frame);
    // Claxon
    frame.element = KLAXON;
    sendToVehicle(frame);

  }
}

/**
 * Método público que realiza una lectura por el socket de comunicación con el 
 * vehículo y obtiene una trama en caso de que el propio vehículo la haya 
 * enviado. La clasifica según el elemento al que hace referencia y procede a su 
 * tratamiento
 * @return Booleano que indica si se ha llevado a cabo una lectura via socket
 */
bool DrivingConnectionManager::checkForVehicleMessages() {
  char bufData[8];


  if (recv(socketDescriptor, bufData, sizeof (bufData), 0) > 0) {

    // Estructura de recepcion
    FrameDriving fdr;
    short aux;
    // Rellenado del buffer
    memcpy(&aux, &bufData[0], sizeof (aux));
    fdr.instruction = static_cast<CommandID> (aux);
    memcpy(&fdr.id_instruccion, &bufData[2], sizeof (fdr.id_instruccion));
    memcpy(&aux, &bufData[4], sizeof (aux));
    fdr.element = static_cast<DeviceID> (aux);
    memcpy(&fdr.value, &bufData[6], sizeof (fdr.value));

    if (fdr.instruction == ACK) {

      printf("Recibido ACK\n");
      if (isCriticalInstruction(fdr.element))
        informResponse(true, fdr.id_instruccion);

    } else if (fdr.instruction == NACK) {

      printf("Recibido NACK\n");
      RtxStruct rtxList = informResponse(false, fdr.id_instruccion);
      for (int i = 0; i < rtxList.numOfMsgs; i++) {
        sendToVehicle((FrameDriving) rtxList.msgs.at(i));
      }

    } else if (fdr.instruction == INFO) {

      if (fdr.element == STEERING_ALARMS || fdr.element == DRIVE_ALARMS) {
        setAlarmsInfo(fdr.element, fdr.value);
      } else { // INFO corriente
        setVehicleInfo(fdr.element, fdr.value);
      }

    }
    return true;
  } else {
    return false;
  }
}

/**
 * Método publico consultor del atributo "socketDescriptor" de la clase 
 * @return Atributo "socketDescriptor" de la clase
 */
int DrivingConnectionManager::getSocketDescriptor() {
  return socketDescriptor;
}

/**
 * Método público consultor del atributo "vehicleInfo" de la clase
 * @param[in] full Indica si se devuelve el atributo completo (señales de actuadores
 * y señalización) o parcial (actuadores únicamente)
 * @return Atributo "vehicleInfo" de la clase
 */
DrivingInfo DrivingConnectionManager::getVehicleInfo(bool full) {
  DrivingInfo ret = vehicleInfo;

  if (full)
    ret.lights = true;
  else
    ret.lights = false;

  return ret;
}

/**
 * Método publico modificador del atributo "vehicleInfo" de la clase con la 
 * información de un dispositivo (actuador o señalización) concreto
 * @param[in] id_device Identificador del dispositivo a modificar
 * @param[in] value Valor de lectura del dispositivo a modificar
 */
void DrivingConnectionManager::setVehicleInfo(DeviceID id_device, short value) {
  switch (id_device) {
    case THROTTLE:
      vehicleInfo.thottle = value;
      break;
    case BRAKE:
      vehicleInfo.brake = value;
      break;
    case HANDBRAKE:
      vehicleInfo.parkingBrake = (bool) value;
      break;
    case STEERING:
      vehicleInfo.steering = value;
      break;
    case GEAR:
      vehicleInfo.gear = value;
      break;
    case MOTOR_RPM:
      vehicleInfo.motorRPM = value;
      break;
    case CRUISING_SPEED:
      vehicleInfo.speed = value;
      break;
    case MOTOR_TEMPERATURE:
      vehicleInfo.motorTemperature = value;
      break;
    case BLINKER_LEFT:
      vehicleInfo.blinkerLeft = (bool) value;
      break;
    case BLINKER_RIGHT:
      vehicleInfo.blinkerRight = (bool) value;
      break;
    case BLINKER_EMERGENCY:
      vehicleInfo.blinkerRight = (bool) value;
      vehicleInfo.blinkerLeft = (bool) value;
      break;
    case KLAXON:
      vehicleInfo.klaxon = (bool) value;
      break;
    case DIPSP:
      vehicleInfo.dipsp = (bool) value;
      break;
    case DIPSS:
      vehicleInfo.dipss = (bool) value;
      break;
    case DIPSR:
      vehicleInfo.dipsr = (bool) value;
      break;
    default:
      break;
  }
}

/**
 * Método público consultor del atributo "countMsg" de la clase utilizado para 
 * llevar el conteo de los mensajes críticos (mecanismo de integridad)
 * @return Atributo "countMsg" de la clase
 */
short DrivingConnectionManager::getCountCriticalMessages() {
  return countMsg;
}

/**
 * Método público modificador del atributo "countMsg" de la clase utilizado para
 * llevar el conteo del los mensajes críticos (mecanismo de integridad). 
 * Contempla que se lleve a cabo según un contador incremental con intervalo 
 * 1..1024
 * @param[in] cont Nuevo valor para el atributo "countMsg"
 */
void DrivingConnectionManager::setCountCriticalMessages(short cont) {
  countMsg = cont;
  // Contador: 1 .. 1024
  if (countMsg == 1025)
    countMsg = 1;
}

/**
 * Método privado modificador de los atributos "driveAlarms" y "steeringAlarms" 
 * de la clase
 * @param[in] element Indica si las alarmas leidas corresponden a las de conduccion
 * o las de direccion
 * @param[in] value Nuevo valor del vector de alarmas a modificar en el atributo
 * correspondiente
 */
void DrivingConnectionManager::setAlarmsInfo(DeviceID element, short value) {
  if (element == DRIVE_ALARMS)
    driveAlarms = value;
  else if (element == STEERING_ALARMS)
    steeringAlarms = value;
}

/**
 * Método público que comprueba si un elemento es crítico y por tanto debe ser 
 * contemplado para llevar a cabo el mecanismo de integridad
 * @param[in] element Elemento de consulta
 * @return Booleano que indica si el elemento es crítico o  no
 */
bool DrivingConnectionManager::isCriticalInstruction(DeviceID element) {
  if (element == RESET
          || element == GEAR
          || element == MT_GEAR
          || element == THROTTLE
          || element == MT_THROTTLE
          || element == CRUISING_SPEED
          || element == HANDBRAKE
          || element == MT_HANDBRAKE
          || element == BRAKE
          || element == MT_BRAKE
          || element == STEERING
          || element == MT_STEERING) {
    return true;
  } else {
    return false;
  }
}

/**
 * Método público que comprueba si un elemento es de tipo MT para activación /
 * desactivación de actuadores
 * @param[in] element Elemento de consulta
 * @return Booleano que indica si el elemento es de tipo MT o no
 */
bool DrivingConnectionManager::isMTCommand(DeviceID element) {
  if (element == RESET
          || element == MT_GEAR
          || element == MT_THROTTLE
          || element == MT_HANDBRAKE
          || element == MT_BRAKE
          || element == MT_STEERING
          || element == MT_BLINKERS
          || element == MT_LIGHTS) {
    return true;
  } else {
    return false;
  }
}

/**
 * Método público que, una vez que un mensaje es considerado crítico, se utiliza 
 * para encolarlo hasta que se reciba el ACK correspondiente o retransmitirlo en 
 * caso de obtener un NACK
 * @param[in] frame Trama a incluir en la cola de mensajes críticos
 */
void DrivingConnectionManager::addToQueue(FrameDriving frame) {
  messageQueue.push_back(frame);
}

/**
 * Método privado para el tratamiento de la cola de mensajes tras la recepcion 
 * de un mensaje de confirmación (ACK) o negación (NACK). En el caso de recibir 
 * un ACK, se desencolan todos los mensajes cuyo campo "id_instruccion" es menor 
 * al que se incluye en la trama del propio ACK. En el caso de recibir un NACK, 
 * se retransmiten todos los mensajes de la cola cuyo campo "id_instruccion" sea
 * menor o igual que el que se incluye en la trama del propio ACK y se elimina
 * el resto
 * @param[in] ack Indica si se ha recibido un ACK (true) o un NACK (false) 
 * @param[in] id_instruction Indica el campo "id_instruccion" (cuenta) del mensaje
 * recibido del vehículo
 * @return Estructura con el numero de mensajes a retransmitir (en caso de
 * haberlos) y una lista de dichos mensajes.
 */
RtxStruct DrivingConnectionManager::informResponse(bool ack, short id_instruction) {

  RtxStruct ret;
  ret.numOfMsgs = 0;

  // Se situa el iterador al principio de la cola
  vector<FrameDriving>::iterator it = messageQueue.begin();

  if (ack) { // ACK

    if (id_instruction == (*it).id_instruccion) { // Primer elemento y requerido coinciden
      // Se elimina el primer elemento
      messageQueue.erase(it);
    } else if (id_instruction > (*it).id_instruccion) { // Confirmacion de varios elementos
      while (id_instruction >= (*it).id_instruccion) {
        messageQueue.erase(it);
      }
    }

  } else { // NACK

    while (it != messageQueue.end()) {
      // Se incluyen en la respuesta todos los  mensajes no confirmados
      // para retransmitir
      ret.numOfMsgs++;
      ret.msgs.push_back((*it));
      it++;
    }

  }
  return ret;
}