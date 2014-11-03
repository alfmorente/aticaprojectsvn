
/** 
 * @file  TraxAHRSModuleDriver.cpp
 * @brief Implementación de la clase "TraxAHRSModuleDriver"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "TraxAHRSModuleDriver.h"

/**
 * Constructor la clase. Almacena las estructuras de almacenaje de información
 * que se vaya leyendo del magnetómetro
 */
TraxAHRSModuleDriver::TraxAHRSModuleDriver() {
  socketDescriptor = -1;
  // Inicio variable recepcion de datos
  oriInfo.heading_status = 0;
  oriInfo.roll = 0;
  oriInfo.pitch = 0;
  oriInfo.heading = 0;
  oriInfo.accX = 0;
  oriInfo.accY = 0;
  oriInfo.accZ = 0;
  oriInfo.gyrX = 0;
  oriInfo.gyrY = 0;
  oriInfo.gyrZ = 0;
}

/**
 * Destructor de la clase
 */
TraxAHRSModuleDriver::~TraxAHRSModuleDriver() {

}

/**
 * Método público que solicita la configuración del dispositivo con los 
 * parámetros establecidos
 */
void TraxAHRSModuleDriver::configureDevice() {
  sendToDevice(kSetDataComponents());
}

/**
 * Método privado que compone un mensaje de tipo KGetModInfo para transmitir al 
 * dispositivo
 * @return Estructura con información para envio de la trama
 */
TraxMsg TraxAHRSModuleDriver::kGetModInfo() {

  TraxMsg retTraxMsg;

  retTraxMsg.byteCount = shortToHexa(5);
  retTraxMsg.packFrame.idFrame = IDFRAME_KGETMODINFO;

  return retTraxMsg;
}

/**
 * Método privado que compone un mensaje de tipo KGetData para transmitir al 
 * dispositivo
 * @return Estructura con informacion para envio de la trama
 */
TraxMsg TraxAHRSModuleDriver::kGetData() {

  TraxMsg retTraxMsg;

  retTraxMsg.byteCount = shortToHexa(5);
  retTraxMsg.packFrame.idFrame = IDFRAME_KGETDATA;

  return retTraxMsg;
}

/**
 * Método privado que compone un mensaje de tipo kSetDataComponents para transmitir al 
 * dispositivo
 * @return Estructura con informacion para envio de la trama
 */
TraxMsg TraxAHRSModuleDriver::kSetDataComponents() {
  TraxMsg retTraxMsg;

  retTraxMsg.byteCount = shortToHexa(10);
  retTraxMsg.packFrame.idFrame = IDFRAME_KSETDATACOMPONENTS;

  // Se solicita heading, roll, pitch, heading status
  retTraxMsg.packFrame.payload.push_back(0x0A);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_HEADING);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_PITCH);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_ROLL);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_HEADING_STATUS);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_ACCX);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_ACCY);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_ACCZ);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_GYRX);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_GYRY);
  retTraxMsg.packFrame.payload.push_back(IDMEASURE_GYRZ);

  return retTraxMsg;
}

/**
 * Método privado que completa el mensaje añadiendo la cola del mensaje y lo 
 * envía al dispositivo
 * @param[in] traxMsg Estructura con información de la trama a enviar
 */

void TraxAHRSModuleDriver::sendToDevice(TraxMsg traxMsg) {
  short len = hexa2short(traxMsg.byteCount);

  char *msg2send = (char *) malloc(len);

  int index = 0;

  // Bytecount
  msg2send[index++] = traxMsg.byteCount[0];
  msg2send[index++] = traxMsg.byteCount[1];

  // ID Frame
  msg2send[index++] = traxMsg.packFrame.idFrame;

  // Payload
  if (len > 5) {
    for (int i = 0; i < len - 5; i++) {
      msg2send[index++] = traxMsg.packFrame.payload[i];
    }
  }

  // CRC16
  short crc16short = crc16ccitt_xmodem((uint8_t *) msg2send, len - 2);
  msg2send[index++] = shortToHexa(crc16short)[0];
  msg2send[index++] = shortToHexa(crc16short)[1];

  if (send(socketDescriptor, msg2send, len, 0) == len) {
    if (traxMsg.packFrame.idFrame == IDFRAME_KGETDATA) rcvResponse();
  } else {
    printf("Problemas en la escritura\n");
  }

  free(msg2send);
}

/**
 * Método privado para la lectura y desencapsulación de una trama procedente del 
 * dispositivo
 */
void TraxAHRSModuleDriver::rcvResponse() {

  // Contador de bytes leidos
  int index = 0;
  // Buffer de recepcion
  //unsigned char* recievedFrame;
  vector< char> recievedFrame;
  //unsigned char byte;
  TraxMsg msgRcv;

  unsigned char byte;

  // Tamaño de la trama
  while (index < 2) {
    if (recv(socketDescriptor, &byte, 1, 0) > 0) {
      recievedFrame.push_back(byte);
      index++;
    } else {
    }
  }

  short tam = hexa2short(recievedFrame);

  recievedFrame.clear();

  recievedFrame.push_back(shortToHexa(tam)[0]);
  recievedFrame.push_back(shortToHexa(tam)[1]);

  // Resto de la trama
  while (index < tam) {
    if (recv(socketDescriptor, &byte, 1, 0) > 0) {
      recievedFrame.push_back(byte);
      index++;
    }
  }

  TraxMsg pkg = mngPacket(recievedFrame);

  if (pkg.checked) {
    oriInfo = unpackPayload(pkg.packFrame.payload);
  }

}

/**
 * Método privado que obtiene la informacion de una trama a partir de los datos 
 * en crudo que se obtienen del dispositivo
 * @param[in] bufferPacket Datos en crudo leidos del dispositivo
 * @return Estructura con información almacenada en tipos obtenida
 */
TraxMsg TraxAHRSModuleDriver::mngPacket(vector<char> bufferPacket) {
  TraxMsg packet;

  packet.checked = false;

  // Desempaqueta del paquete

  // Tamaño
  packet.byteCount.push_back(bufferPacket[0]);
  packet.byteCount.push_back(bufferPacket[1]);
  short tam = hexa2short(packet.byteCount);

  // Frame ID
  packet.packFrame.idFrame = bufferPacket[2];

  // Payload
  for (int i = 0; i < (tam - 5); i++) {
    packet.packFrame.payload.push_back(bufferPacket[i + 3]);
  }

  // CRC16
  packet.crc.push_back(bufferPacket[tam - 2]);
  packet.crc.push_back(bufferPacket[tam - 1]);

  char *auxBuff = (char *) malloc(tam - 2);
  for (int i = 0; i < tam - 2; i++) {
    auxBuff[i] = bufferPacket[i];
  }
  if ((short) (crc16ccitt_xmodem((uint8_t *) auxBuff, tam - 2)) == hexa2short(packet.crc)) {
    // Recepcion correcta por CRC16
    packet.checked = true;
  } else {
    printf("Checksum error. Frame discarted\n");
  }
  free(auxBuff);

  return packet;

}

/**
 * Método privado que obtiene la información de los datos en crudo del fragmento 
 * "payload" donde se almacenan los datos efectivos de la trama
 * @param[in] payload Datos en crudo del campo "payload" de la trama
 * @return Estructura con las medidas obtenidas de los datos "payload" de la 
 * trama
 */
TraxMeasurement TraxAHRSModuleDriver::unpackPayload(std::vector<char> payload) {
  int tam = payload[0];
  int numData = 0;
  int index = 1;
  // char *bufFloat = (char *)malloc(4);
  vector<char> bufFloat;

  TraxMeasurement measureDev;

  while (numData < tam) {
    switch (payload[index]) {
      case IDMEASURE_HEADING:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.heading = degrees2radians(hexa2float(bufFloat));
        index += 5;
        break;
      case IDMEASURE_PITCH:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.pitch = degrees2radians(hexa2float(bufFloat));
        index += 5;
        break;
      case IDMEASURE_ROLL:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.roll = degrees2radians(hexa2float(bufFloat));
        index += 5;
        break;
      case IDMEASURE_HEADING_STATUS:
        measureDev.heading_status = payload[index + 1];
        index += 2;
        break;
      case IDMEASURE_ACCX:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.accX = hexa2float(bufFloat);
        index += 5;
        break;
      case IDMEASURE_ACCY:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.accY = hexa2float(bufFloat);
        index += 5;
        break;
      case IDMEASURE_ACCZ:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.accZ = hexa2float(bufFloat);
        index += 5;
        break;
      case IDMEASURE_GYRX:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.gyrX = hexa2float(bufFloat);
        index += 5;
        break;
      case IDMEASURE_GYRY:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.gyrY = hexa2float(bufFloat);
        index += 5;
        break;
      case IDMEASURE_GYRZ:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.gyrZ = hexa2float(bufFloat);
        index += 5;
        break;
      default:
        break;
    }
    numData++;
  }
  return measureDev;
}

/**
 * Método público consultor del atributo "oriInfo" de la clase que almacena la 
 * información de la última lectura de los dispositivos
 * @return Atributo "oriInfo" de la clase
 */
TraxMeasurement TraxAHRSModuleDriver::getInfo() {
  return oriInfo;
}

/**
 * Método público para el envío de mensaje KGetData para la solicitud de 
 * información al dispositivo
 * @return Booleano que indica cuando la transmision llega a su fin 
 */
bool TraxAHRSModuleDriver::getData() {
  sendToDevice(kGetData());
  return true;
}

/**
 * Función de conversion de tipos. Convierte un vector de bytes en un float
 * @param[in] buffer[in] Datos en crudo a convertir
 * @return Float resultado de la conversion
 */
float TraxAHRSModuleDriver::hexa2float(vector<char> buffer) {

  union {
    float value;
    unsigned char buffer[4];

  } floatUnion;

  floatUnion.buffer[0] = buffer[3];
  floatUnion.buffer[1] = buffer[2];
  floatUnion.buffer[2] = buffer[1];
  floatUnion.buffer[3] = buffer[0];

  return floatUnion.value;
}

/**
 * Función de conversion de tipos. Convierte un vector de bytes en un int
 * @param[in] buffer[in] Datos en crudo a convertir
 * @return Int resultado de la conversion
 */
int TraxAHRSModuleDriver::hexa2int(std::vector<unsigned char> buffer) {

  union {
    int value;
    unsigned char buffer[4];

  } intUnion;

  intUnion.buffer[0] = buffer[3];
  intUnion.buffer[1] = buffer[2];
  intUnion.buffer[2] = buffer[1];
  intUnion.buffer[3] = buffer[0];

  return intUnion.value;
}

/**
 * Función de conversion de tipos. Convierte un vector de bytes en un short
 * @param[in] buffer[in] Datos en crudo a convertir
 * @return Short resultado de la conversion
 */
short TraxAHRSModuleDriver::hexa2short(vector<char> buffer) {

  union {
    short value;
    unsigned char buffer[2];

  } shortUnion;

  shortUnion.buffer[0] = buffer[1];
  shortUnion.buffer[1] = buffer[0];

  return shortUnion.value;
}

/**
 * Función de conversion de tipos. Convierte un vector de bytes en un double
 * @param[in] buffer[in] Datos en crudo a convertir
 * @return Double resultado de la conversion
 */
double TraxAHRSModuleDriver::hexa2double(std::vector<unsigned char> buffer) {

  union {
    double value;
    unsigned char buffer[8];
  } doubleUnion;

  doubleUnion.buffer[0] = buffer[7];
  doubleUnion.buffer[1] = buffer[6];
  doubleUnion.buffer[2] = buffer[5];
  doubleUnion.buffer[3] = buffer[4];
  doubleUnion.buffer[4] = buffer[3];
  doubleUnion.buffer[5] = buffer[2];
  doubleUnion.buffer[6] = buffer[1];
  doubleUnion.buffer[7] = buffer[0];

  return doubleUnion.value;
}

/**
 * Función de conversion de tipos. Convierte un dato de tipo short a una cadena
 * de bytes
 * @param[in] buffer[in] Dato a convertir en bytes
 * @return Vector de bytes resultado de la conversion
 */
vector<char> TraxAHRSModuleDriver::shortToHexa(short s) {
  char *buf = (char *) malloc(2);
  vector<char> out;
  memcpy(buf, &s, 2);
  out.push_back(buf[1]);
  out.push_back(buf[0]);
  free(buf);
  return out;

}

/**
 * Método privado de conversión de tipos. Convierte un valor en grados a
 * radianes
 * @param[in] value Valor a convertir
 * @return Resultado de la conversión
 */
float TraxAHRSModuleDriver::degrees2radians(float value){
  return (value * M_PI)/180;
}
