
/** 
 * @file  TraxAHRSModuleDriver.h
 * @brief Declara el tipo de la clase "TraxAHRSModuleDriver"
 * - La clase implementa la comunicación con el dispositivo magnetómetro TRAX
 * AHRS Module encargado de obtener la orientación del vehículo
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup MagnetometerDriver
 * @{
 */

#ifndef TRAXAHRSMODULEDRIVER_H
#define	TRAXAHRSMODULEDRIVER_H

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>
#include <complex>
#include <vector>
#include "constant.h"
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include "crc16calc.h"
#include "SocketDriver.h"

using namespace std;

/**
 * \class TraxAHRSModuleDriver
 * \brief Clase que representa al driver de comunicación con el dispositivo
 * magnetómetro Trax AHRS Module
 */
class TraxAHRSModuleDriver:public SocketDriver {
private:
  TraxMeasurement oriInfo;
  TraxMsg kGetModInfo();
  TraxMsg kGetData();
  TraxMsg kSetDataComponents();
  void sendToDevice(TraxMsg);
  void rcvResponse();
  TraxMsg mngPacket(std::vector< char>);
  TraxMeasurement unpackPayload(std::vector<char>);
  float hexa2float(std::vector<char>);
  double hexa2double(std::vector<unsigned char>);
  int hexa2int(std::vector<unsigned char>);
  short hexa2short(std::vector<char>);
  std::vector<char> shortToHexa(short);
  float degrees2radians(float);
public:
  TraxAHRSModuleDriver();
  ~TraxAHRSModuleDriver();
  void configureDevice();
  bool getData();
  TraxMeasurement getInfo();
};

#endif	/* TRAXAHRSMODULEDRIVER_H */

/**
 * @}
 */

