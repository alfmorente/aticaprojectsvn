
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
  canal = -1;
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
 * Método público que inicia la conexión con el dispositivo
 * @return Booleano que indica si la conexión se ha realizado con éxito
 */
bool TraxAHRSModuleDriver::connectToDevice() {

  char * serial_name = (char *) "/dev/ttyUSB1";

  canal = open(serial_name, O_RDWR | O_NOCTTY | O_NDELAY);

  if (canal < 0) {
    return false;
  } else {

    tcgetattr(canal, &oldtio);
    bzero(&newtio, sizeof (newtio));
    newtio.c_cflag = B38400 | CS8 | CLOCAL | CREAD;
    newtio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
    newtio.c_oflag = 0;
    newtio.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

    newtio.c_cc[VINTR] = 0; /* Ctrl-c */
    newtio.c_cc[VQUIT] = 0; /* Ctrl-\ */
    newtio.c_cc[VERASE] = 0; /* del */
    newtio.c_cc[VKILL] = 0; /* @ */
    newtio.c_cc[VEOF] = 4; /* Ctrl-d */
    newtio.c_cc[VTIME] = 0; /* temporizador entre caracter, no usado */
    newtio.c_cc[VMIN] = 1; /* bloqu.lectura hasta llegada de caracter. 1 */
    newtio.c_cc[VSWTC] = 0; /* '\0' */
    newtio.c_cc[VSTART] = 0; /* Ctrl-q */
    newtio.c_cc[VSTOP] = 0; /* Ctrl-s */
    newtio.c_cc[VSUSP] = 0; /* Ctrl-z */
    newtio.c_cc[VEOL] = 0; /* '\0' */
    newtio.c_cc[VREPRINT] = 0; /* Ctrl-r */
    newtio.c_cc[VDISCARD] = 0; /* Ctrl-u */
    newtio.c_cc[VWERASE] = 0; /* Ctrl-w */
    newtio.c_cc[VLNEXT] = 0; /* Ctrl-v */
    newtio.c_cc[VEOL2] = 0; /* '\0' */

    tcflush(canal, TCIFLUSH);
    tcsetattr(canal, TCSANOW, &newtio);
  }

  return true;
}

/**
 * Método público que cierra la conexion con el dispositivo
 */
void TraxAHRSModuleDriver::disconnectDevice() {
  close(canal);
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

  if (write(canal, msg2send, len) == len) {
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
    if (read(canal, &byte, 1) > 0) {
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
    if (read(canal, &byte, 1) > 0) {
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
        measureDev.heading = hexa2float(bufFloat);
        index += 5;
        break;
      case IDMEASURE_PITCH:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.pitch = hexa2float(bufFloat);
        index += 5;
        break;
      case IDMEASURE_ROLL:
        bufFloat.push_back(payload[index + 1]);
        bufFloat.push_back(payload[index + 2]);
        bufFloat.push_back(payload[index + 3]);
        bufFloat.push_back(payload[index + 4]);
        measureDev.roll = hexa2float(bufFloat);
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

