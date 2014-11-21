
/** 
 * @file  XSensMTi700Driver.h
 * @brief Declara el tipo de la clase "XSensMTi700Driver"
 * - La clase implementa la comunicacion con el dispositivo X-Sens MTi-G 700 
 * encargado de obtener la posición y orientación del vehículo
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup INSGPSDriver
 * @{
 */

#ifndef XSENSMTI700DRIVER_H
#define	XSENSMTI700DRIVER_H

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>
#include <complex>
#include <pthread.h>
#include <vector>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include "constant.h"
#include "SocketDriver.h"

using namespace std;

/**
 * \class XSensMTi700Driver
 * \brief Clase que representa al driver de comunicación con el dispositivo
 * GPS+IMU XSens MTi-G 700
 */
class XSensMTi700Driver:public SocketDriver {
private:
  GPSINSInfo posOriInfo;
  void sendToDevice(XsensMsg);
  void waitForAck(unsigned char);
  unsigned char calcChecksum(XsensMsg);
  bool isCheckSumOK(std::vector<unsigned char>);
  XsensMsg goToConfig();
  XsensMsg goToMeasurement();
  XsensMsg setOutPutConfiguration();
  float hexa2float(std::vector<unsigned char>);
  double hexa2double(std::vector<unsigned char>);
  int hexa2int(std::vector<unsigned char>);
  float degrees2radians(float);
  void packetMng(dataPacketMT2);
  bool frameMng(std::vector<unsigned char>);
public:
  XSensMTi700Driver();
  ~XSensMTi700Driver();
  void configureDevice();
  bool getData();
  GPSINSInfo getInfo();
};

#endif	/* XSENSMTI700DRIVER_H */

/**
 * @}
 */
