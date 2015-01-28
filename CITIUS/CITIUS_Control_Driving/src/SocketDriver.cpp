/** 
 * @file  SocketDriver.cpp
 * @brief Implementacion de la clase "SocketDriver"
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "SocketDriver.h"

/**
 * Método público que realiza la inicialización y conexión del socket de 
 * comunicación con el vehículo
 * @return Booleano que indica si la conexión ha sido posible
 */
bool SocketDriver::doConnect(int device) {
  string ip = getValueFromConfig(CONFIG_FILE_IP_NAME,device);
  if (ip == "") return false;
  string port = getValueFromConfig(CONFIG_FILE_PORT_NAME,device);
  if (port == "") return false;
  if ((he = gethostbyname(ip.c_str())) == NULL) {
    return false;
  }
  if ((socketDescriptor = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
    return false;
  }
  server.sin_family = AF_INET;
  server.sin_port = htons(atoi(port.c_str()));
  server.sin_addr = *((struct in_addr *) he->h_addr);
  bzero(&(server.sin_zero), 8);
  if (connect(socketDescriptor, (struct sockaddr *) &server, sizeof (struct sockaddr)) == -1) {
    return false;
  }
  if (fcntl(socketDescriptor, F_SETFL, fcntl(socketDescriptor, F_GETFL) | O_NONBLOCK) < 0) {
    printf("Imposible establecer socket como no bloqueante en operaciones L/E\n");
  }
  return true;
}

/**
 * Método público que realiza la desconexión del vehículo mediante la liberación 
 * del socket de comunicación
 */
void SocketDriver::disconnect() {
  shutdown(socketDescriptor, 2);
  close(socketDescriptor);
}

/**
 * Método privado que busca en el fichero de configuración el valor del campo
 * de configuración que recibe como parámetro
 * @param parameter Identificador del campo a buscar en el fichero
 * @return String con el resultado de la búsqueda
 */
string SocketDriver::getValueFromConfig(string parameter, int device){
  int pos;
  string cadena, parametro, value = "";
  bool found = false;
  ifstream fichero;
  if (device == DEVICE_ELECTRIC || device == DEVICE_DRIVING) {
    fichero.open("/home/ugv/catkin_ws/src/CITIUS_Control_Driving/bin/socket.conf");
  } else if (device == DEVICE_XSENS) {
    fichero.open("/home/ugv/catkin_ws/src/CITIUS_Control_Driving/bin/socket_INSGPS.conf");
  } else if (device == DEVICE_AHRS) {
    fichero.open("/home/ugv/catkin_ws/src/CITIUS_Control_Driving/bin/socket_MAGN.conf");
  }
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
