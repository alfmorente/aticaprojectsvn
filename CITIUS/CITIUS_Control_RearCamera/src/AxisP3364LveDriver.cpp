
/** 
 * @file  AxisP3364LveDriver.cpp
 * @brief Implementación de la clase "AxisP3364LveDriver"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "AxisP3364LveDriver.h"

/** 
 * Constructor de la clase
 */
AxisP3364LveDriver::AxisP3364LveDriver() {
  pan = 0;
  tilt = 0;
  zoom = 0;
}

/** 
 * Destructor de la clase
 */
AxisP3364LveDriver::~AxisP3364LveDriver() {

}

/**
 * Método privado que busca en el fichero de configuración el valor del campo
 * de configuración que recibe como parámetro
 * @param parameter Identificador del campo a buscar en el fichero
 * @return String con el resultado de la búsqueda
 */
string AxisP3364LveDriver::getValueFromConfig(string parameter) {

  int pos;
  string cadena, parametro, value = "";
  bool found = false;
  ifstream fichero;
  fichero.open("socket.conf");

  if (!fichero.is_open()) {
    return "";
  }

  while (!fichero.eof() && !found) {
    getline(fichero, cadena);
    if (cadena[0] != '#' && cadena[0] != NULL) {
      pos = cadena.find(":");
      if (pos != -1) {
        parametro = cadena.substr(0, pos);
        if (parametro == parameter) {
          value = cadena.substr(pos + 1);
          while (isspace(value[0])) {
            value = value.substr(1);
          }
          found = true;
        }
      }
    }
  }
  fichero.close();

  return value;

}

/**
 * Método público que comprueba la disponibilidad de la cámara IP en la red 
 * @return Booleano que indica si la cámara está operativa para ser utilizada
 */
bool AxisP3364LveDriver::checkConnection() {

  string ipFile = getValueFromConfig(CONFIG_FILE_IP_NAME);
  if (ipFile == "") return false;

  ip_address = ipFile.c_str();

  string portFile = getValueFromConfig(CONFIG_FILE_PORT_NAME);
  if (portFile == "") return false;

  port = atoi(portFile.c_str());

  if ((he = gethostbyname(ip_address)) == NULL) {
    return false;
  }

  if ((socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    close(socketDescriptor);
    usleep(500);
    return false;
  }
  server.sin_family = AF_INET;
  server.sin_port = htons(port);
  server.sin_addr = *((struct in_addr *) he->h_addr);

  bzero(&(server.sin_zero), 8);

  if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) == -1) {
    close(socketDescriptor);
    usleep(500);
    return false;
  }
  close(socketDescriptor);
  usleep(500);
  return true;

}

/**
 * Método público que utiliza los párametros para el montaje de un comando de 
 * control para la cámara y lo transmite. Recibe la respuesta de la cámara con 
 * el estado actual de las variables y actualiza el registro del estado
 * @param[in] order Componente sobre el que transmitir la orden
 * @param[in] value Nuevo para a transmitir al componente
 * @return Booleano que indica si la orden se ha ejecutado con éxito
 */
bool AxisP3364LveDriver::sendSetToDevice(short order, float value) {

  if ((he = gethostbyname(ip_address)) == NULL) {
    return false;
  }

  if ((socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    close(socketDescriptor);
    usleep(500);
    return false;
  }

  server.sin_family = AF_INET;
  server.sin_port = htons(port);
  server.sin_addr = *((struct in_addr *) he->h_addr);

  bzero(&(server.sin_zero), 8);

  if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) == -1) {
    close(socketDescriptor);
    usleep(500);
    return false;

  }
  stringstream stream;
  stream << "GET http://" /*<< AUTH_CAM_USER << "@" << AUTH_CAM_PASS << ":" */ << ip_address << PTZ_ROUTE;
  switch (order) {
    case ORDER_ZOOM:
      stream << "zoom=";
      break;
    case ORDER_TILT:
      stream << "tilt=";
      break;
    case ORDER_PAN:
      stream << "pan=";
      break;
    default:
      stream.str().clear();
      break;
  }

  if (stream.str().size() == 0) {
    close(socketDescriptor);
    usleep(500);
    return false;
  }

  stream << value << "\nConnection: keep-alive\r\n";

  int nBytesSent = send(socketDescriptor, stream.str().c_str(), strlen(stream.str().c_str()), 0);

  if (nBytesSent < 0) {
    close(socketDescriptor);
    usleep(500);
    return false;
  }

  // Lectura y obtencion del ack
  char respuesta[256];

  int nBytesRead = 0;
  while (nBytesRead == 0) {
    nBytesRead = recv(socketDescriptor, respuesta, 256, 0);
  }
  close(socketDescriptor);
  usleep(500);
  LensPosition lenspos = getPosition();

  if (lenspos.state) {
    close(socketDescriptor);
    usleep(500);
    return true;
  }
  close(socketDescriptor);
  usleep(500);
  return false;
}

/**
 * Método público que solicita el valor de las variables sobre las que se puede 
 * actuar 
 * @return Estructura con el valor de las variables leidas en caso de que se 
 * haya obtenido la información con éxito
 */
LensPosition AxisP3364LveDriver::getPosition() {

  LensPosition pos;
  pos.state = false;
  pos.pan = 0;
  pos.tilt = 0;
  pos.zoom = 0;

  socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);

  if (socketDescriptor >= 0) {

    if ((he = gethostbyname(ip_address)) != NULL) {
      server.sin_family = AF_INET;
      server.sin_port = htons(port);
      server.sin_addr = *((struct in_addr *) he->h_addr);

      bzero(&(server.sin_zero), 8);

      if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) != -1) {
        //printf("Socket d: %d\n",socketDescriptor);
        stringstream stream;
        stream << "GET http://" << AUTH_CAM_USER << "@" << AUTH_CAM_PASS << ":" << ip_address << PTZ_ROUTE << "query=position\r\n";
        //printf("Comando generado: %s", stream.str().c_str());

        int nBytesSent = send(socketDescriptor, stream.str().c_str(), strlen(stream.str().c_str()), 0);

        if (nBytesSent < 0) {
          printf("Error en la escritura\n");
          pos.state = false;
        }

        // Lectura y obtencion de los datos
        char respuesta[256];
        int nBytesRead = recv(socketDescriptor, respuesta, 256, 0);
        respuesta[nBytesRead] = '\0';

        if (nBytesRead > 0) {

          pos.state = true;
          pos.pan = extractPan(respuesta);
          pos.tilt = extractTilt(respuesta);
          pos.zoom = extractZoom(respuesta);

        }

      }

    }
  }
  close(socketDescriptor);
  usleep(500);
  return pos;
}

/**
 * Método privado que obtiene el valor de Zoom del String que devuelve la cámara 
 * cuando se realiza una lectura del estado de sus variables
 * @param[in] cadena String con la respuesta integra de la cámara
 * @return Valor obtenido tras la búsqueda de zoom en la cadena
 */
float AxisP3364LveDriver::extractZoom(char cadena[256]) {
  string resp = cadena;
  int index = resp.find("zoom=");
  string value;
  for (unsigned int i = index + 5; i < resp.size(); i++) {
    if (resp.at(i) == '\r') {
      i = resp.size();
    } else {
      value.push_back(resp.at(i));
    }
  }
  return (float) atof(value.c_str());
}

/**
 * Método privado que obtiene el valor de Tilt del String que devuelve la cámara 
 * cuando se realiza una lectura del estado de sus variables
 * @param[in] cadena String con la respuesta integra de la cámara
 * @return Valor obtenido tras la busqueda de tilt en la cadena
 */
float AxisP3364LveDriver::extractTilt(char cadena[256]) {
  string resp = cadena;
  int index = resp.find("tilt=");
  string value;
  for (unsigned int i = index + 5; i < resp.size(); i++) {
    if (resp.at(i) == '\r') {
      i = resp.size();
    } else {
      value.push_back(resp.at(i));
    }
  }
  return (float) atof(value.c_str());
}

/**
 * Método privado que obtiene el valor de Pan del String que devuelve la cámara 
 * cuando se realiza una lectura del estado de sus variables
 * @param[in] cadena String con la respuesta integra de la cámara
 * @return Valor obtenido tras la busqueda de pan en la cadena
 */
float AxisP3364LveDriver::extractPan(char cadena[256]) {
  string resp = cadena;
  int index = resp.find("pan=");
  string value;
  for (unsigned int i = index + 4; i < resp.size(); i++) {
    if (resp.at(i) == '\r') {
      i = resp.size();
    } else {
      value.push_back(resp.at(i));
    }
  }
  return (float) atof(value.c_str());
}

/**
 * Método público consultor del atributo "pan" de la clase que registra el valor 
 * de Pan de la última lectura 
 * @return Atributo "pan" de la clase
 */
float AxisP3364LveDriver::getPan() {
  return pan;
}

/**
 * Método público consultor del atributo "tilt" de la clase que registra el valor 
 * de Pan de la última lectura 
 * @return Atributo "tilt" de la clase
 */
float AxisP3364LveDriver::getTilt() {
  return tilt;
}

/**
 * Método público consultor del atributo "zoom" de la clase que registra el valor 
 * de Pan de la última lectura 
 * @return Atributo "zoom" de la clase
 */
float AxisP3364LveDriver::getZoom() {
  return zoom;
}

/**
 * Método público modificador del atributo "pan" de clase utilizado tras una 
 * lectura del estado de la cámara
 * @param[in] newPan Nuevo valor del atributo "pan"
 */
void AxisP3364LveDriver::setPan(float newPan) {
  pan = newPan * CONV_FROM_CAMERA; // *5000/100 Conversiona  formato cámara
}

/**
 * Método público modificador del atributo "tilt" de clase utilizado tras una 
 * lectura del estado de la cámara
 * @param[in] newPan Nuevo valor del atributo "tilt"
 */
void AxisP3364LveDriver::setTilt(float newTilt) {
  tilt = newTilt * CONV_FROM_CAMERA; // *5000/100 Conversiona  formato cámara
}

/**
 * Método público modificador del atributo "zoom" de clase utilizado tras una 
 * lectura del estado de la cámara
 * @param[in] newPan Nuevo valor del atributo "zoom"
 */
void AxisP3364LveDriver::setZoom(float newZoom) {
  zoom = newZoom * CONV_FROM_CAMERA; // *5000/100 Conversiona  formato cámara
}
