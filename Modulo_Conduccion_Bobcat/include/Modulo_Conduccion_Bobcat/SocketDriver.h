
/** 
 * @file  SocketDriver.h
 * @brief Declara el tipo de la clase "SocketDriver"
 * - La clase se presenta como la superclase de la cual hereda cada driver que
 * gestione un dispositivo via socket
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup CommonDriver
 * @{
 */

#ifndef SOCKETDRIVER_H
#define	SOCKETDRIVER_H

#include "constantes.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <fstream>

using namespace std;

/**
 * \class SocketDriver
 * \brief Superclase con métodos comunes a todos los drivers socket
 */
class SocketDriver {
protected:
  int socketDescriptor;
  struct hostent *he;
  struct sockaddr_in server;
  string getValueFromConfig(string parameter, int device);
public:
  bool doConnect(int device);
  void disconnect();
  
};

#endif	/* SOCKETDRIVER_H */

/**
 * @}
 */