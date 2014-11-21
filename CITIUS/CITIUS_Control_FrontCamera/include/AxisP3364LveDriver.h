
/** 
 * @file  AxisP3364LveDriver.h
 * @brief Declara el tipo de la clase "AxisP3364LveDriver"
 * - La clase implementa la comunicacion con el dispositivo AXIS P3364-LVe que 
 * se utiliza como camara de apoyo a la conduccion.
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup FrontCameraDriver 
 * @{
 */

#ifndef AXISP3364LVEDRIVER_H
#define	AXISP3364LVEDRIVER_H

#include <cstdlib>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sstream>
#include <fstream>
#include "constant.h"

using namespace std;

/**
 * \class AxisP3364LveDriver
 * \brief Clase que representa al driver de comunicación con el dispositivo
 * cámara de apoyo a la conducción AXIS P3364Lve
 */
class AxisP3364LveDriver {
public:
  AxisP3364LveDriver();
  ~AxisP3364LveDriver();
  bool checkConnection();
  string getValueFromConfig(string parameter);
  bool sendSetToDevice(short order, float value);
  LensPosition getPosition();
  float getPan();
  float getTilt();
  float getZoom();
  void setPan(float);
  void setTilt(float);
  void setZoom(float);
private:
  int socketDescriptor;
  struct hostent *he;
  struct sockaddr_in server;
  const char *ip_address;
  int port;
  float pan;
  float tilt;
  float zoom;
  float extractZoom(char[]);
  float extractTilt(char[]);
  float extractPan(char[]);
};

#endif	/* AXISP3364LVEDRIVER_H */

/**
 * @}
 */