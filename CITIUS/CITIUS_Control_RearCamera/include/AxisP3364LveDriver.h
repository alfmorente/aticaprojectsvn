
/** 
 * @file  AxisP3364LveDriver.h
 * @brief Declara el tipo de la clase "AxisP3364LveDriver"
 * - La clase implementa la comunicacion con el dispositivo AXIS P3364-LVe que 
 * se utiliza como camara de apoyo a la conduccion.
 * @author: Carlos Amores
 * @date: 2013, 2014
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

#define IP_CAMERA "192.168.24.120"
#define PORT_CAMERA 80

#define AUTH_CAM_USER "root"
#define AUTH_CAM_PASS "usv"

#define PTZ_ROUTE "/axis-cgi/com/ptz.cgi?"

#define ORDER_ZOOM 0
#define ORDER_PAN 1
#define ORDER_TILT 2

typedef struct {
  bool state; /// Valor del estado (actualizado o no) de lectura de variables
  float zoom; /// Valor de zoom
  float pan; /// Valor de pan
  float tilt; /// Valor de tilt
} LensPosition;

using namespace std;

#endif	/* AXISP3364LVEDRIVER_H */

class AxisP3364LveDriver {
public:

  AxisP3364LveDriver();
  ~AxisP3364LveDriver();

  // Conexion con la camara (comprobacion de disponibilidad)
  bool checkConnection();

  // Envio / Requerimiento de informacion
  bool sentSetToDevice(short order, float value);
  LensPosition getPosition();

  // Getter y setter necesarios
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

  // Propiedades de control
  float pan;
  float tilt;
  float zoom;

  float extractZoom(char[]);
  float extractTilt(char[]);
  float extractPan(char[]);

};