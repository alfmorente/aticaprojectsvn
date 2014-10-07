
/** 
 * @file  AxisP3364LveDriver.cpp
 * @brief Implementacion de la clase "AxisP3364LveDriver"
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "AxisP3364LveDriver.h"

/** Constructor de la clase*/
AxisP3364LveDriver::AxisP3364LveDriver() {
  pan = 0;
  tilt = 0;
  zoom = 0;
}

/** Destructor de la clase*/
AxisP3364LveDriver::~AxisP3364LveDriver() {

}

/**
 * Comprueba la disponibilidad de la camara IP en la red 
 * @return Booleano que indica si la camara esta operativa para ser utilizada
 */
bool AxisP3364LveDriver::checkConnection() {
  socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);

  if (socketDescriptor < 0) {
    return false;
  } else {

    if ((he = gethostbyname(IP_CAMERA)) == NULL) {
      return false;
    }

    if ((socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      return false;
    }

    server.sin_family = AF_INET;
    server.sin_port = htons(PORT_CAMERA);
    server.sin_addr = *((struct in_addr *) he->h_addr);

    bzero(&(server.sin_zero), 8);

    if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) == -1) {
      return false;
    } else {
      return true;
    }

    return false;
  }
}

/**
 * Utiliza los parametros para el montaje de un comando de control para la
 * camara y lo transmite. Recibe la respuesta de la camara con el estado actual
 * de las variables y actualiza el registro del estado
 * @param[in] order Componente sobre el que transmitir la orden
 * @param[in] value Nuevo para a transmitir al componente
 * @return Booleano que indica si la orden se ha ejecutado con exito
 */
bool AxisP3364LveDriver::sentSetToDevice(short order, float value) {
  socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);

  if (socketDescriptor < 0) {
    return false;
  } else {

    if ((he = gethostbyname(IP_CAMERA)) == NULL) {
      return false;
    }

    if ((socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      return false;
    }

    server.sin_family = AF_INET;
    server.sin_port = htons(PORT_CAMERA);
    server.sin_addr = *((struct in_addr *) he->h_addr);

    bzero(&(server.sin_zero), 8);

    if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) == -1) {
      return false;
    } else {

      stringstream stream;
      stream << "GET http://" /*<< AUTH_CAM_USER << "@" << AUTH_CAM_PASS << ":" */ << IP_CAMERA << PTZ_ROUTE;
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
        return false;
      }

      stream << value << "\nConnection: keep-alive\r\n";


      int nBytesSent = send(socketDescriptor, stream.str().c_str(), strlen(stream.str().c_str()), 0);

      if (nBytesSent < 0) {
        return false;
      }

      // Lectura y obtencion del ack
      char respuesta[256];

      int nBytesRead = 0;
      while (nBytesRead == 0) {
        nBytesRead = recv(socketDescriptor, respuesta, 256, 0);
      }

      LensPosition lenspos = getPosition();

      if (lenspos.state) {
        return true;
      } else {
        return false;
      }
      return true;
    }
    return false;
  }

}

/**
 * Solicita el valor de las variables sobre las que se puede actuar 
 * @return Estructura con el valor de las variables leidas en caso de que se 
 * haya obtenido la informacion con exito
 */
LensPosition AxisP3364LveDriver::getPosition() {

  LensPosition pos;
  pos.state = false;
  pos.pan = 0;
  pos.tilt = 0;
  pos.zoom = 0;

  socketDescriptor = socket(AF_INET, SOCK_STREAM, 0);

  if (socketDescriptor >= 0) {

    if ((he = gethostbyname(IP_CAMERA)) != NULL) {
      server.sin_family = AF_INET;
      server.sin_port = htons(PORT_CAMERA);
      server.sin_addr = *((struct in_addr *) he->h_addr);

      bzero(&(server.sin_zero), 8);

      if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) != -1) {

        stringstream stream;
        stream << "GET http://" << AUTH_CAM_USER << "@" << AUTH_CAM_PASS << ":" << IP_CAMERA << PTZ_ROUTE << "query=position\r\n";
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

  return pos;
}

/**
 * Obtiene el valor de Zoom del String que devuelve la camara cuando se realiza
 * una lectura del estado de sus variables
 * @param[in] cadena String con la respuesta integra de la camara
 * @return Valor obtenido tras la busqueda de zoom en la cadena
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
 * Obtiene el valor de Tilt del String que devuelve la camara cuando se realiza
 * una lectura del estado de sus variables
 * @param[in] cadena String con la respuesta integra de la camara
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
 * Obtiene el valor de Pan del String que devuelve la camara cuando se realiza
 * una lectura del estado de sus variables
 * @param[in] cadena String con la respuesta integra de la camara
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
 * Consultor del atributo "pan" de la clase que registra el valor de Pan 
 * de la ultima lectura 
 * @return Atributo "pan" de la clase
 */
float AxisP3364LveDriver::getPan() {
  return pan;
}

/**
 * Consultor del atributo "tilt" de la clase que registra el valor de Tilt
 * de la ultima lectura
 * @return Atributo "tilt" de la clase
 */
float AxisP3364LveDriver::getTilt() {
  return tilt;
}

/**
 * Consultor del atributo "zoom" de la clase que regustra el valor de Zoom
 * de la ultima lectura
 * @return Atributo "zoom" de la clase
 */
float AxisP3364LveDriver::getZoom() {
  return zoom;
}

/**
 * Modificador del atributo "pan" de clase utilizado tras una lectura del estado
 * de la camara
 * @param[in] newPan Nuevo valor del atributo "pan"
 */
void AxisP3364LveDriver::setPan(float newPan) {
  pan = newPan * 50; // *5000/100 Conversiona  formato camara
}

/**
 * Modificador del atributo "tilt" de clase utilizado tras una lectura del 
 * estado de la camara
 * @param[in] newPan Nuevo valor del atributo "tilt"
 */
void AxisP3364LveDriver::setTilt(float newTilt) {
  tilt = newTilt * 50; // *5000/100 Conversiona  formato camara
}

/**
 * Modificador del atributo "zoom" de clase utilizado tras una lectura del 
 * estado de la camara
 * @param[in] newPan Nuevo valor del atributo "zoom"
 */
void AxisP3364LveDriver::setZoom(float newZoom) {
  zoom = newZoom * 50; // *5000/100 Conversiona  formato camara
}
