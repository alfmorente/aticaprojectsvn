
/** 
 * @file  XSensMTi700Driver.h
 * @brief Declara el tipo de la clase "XSensMTi700Driver"
 * - La clase implementa la comunicacion con el dispositivo X-Sens MTi-G 700 
 * encargado de obtener la posición y orientación del vehículo
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef XSENSMTI700DRIVER_H
#define	XSENSMTI700DRIVER_H

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>
#include <complex>
#include <pthread.h>
#include <vector>
#include "constant.h"

using namespace std;

/**
 * /class XSensMTi700Driver
 * /brief Clase que representa al driver de comunicación con el dispositivo
 * GPS+IMU XSens MTi-G 700
*/
class XSensMTi700Driver {
private:
  // Datos recibidos
  GPSINSInfo posOriInfo;
  // Puerto serie
  struct termios newtio, oldtio;
  int canal;
  // Operaciones a bajo nivel
  void sendToDevice(XsensMsg);
  void waitForAck(unsigned char);
  unsigned char calcChecksum(XsensMsg);
  bool isCheckSumOK(std::vector<unsigned char>);
  XsensMsg goToConfig();
  XsensMsg goToMeasurement();
  XsensMsg setOutPutConfiguration();
  // Conversion de tipos
  float hexa2float(std::vector<unsigned char>);
  double hexa2double(std::vector<unsigned char>);
  int hexa2int(std::vector<unsigned char>);
  // Rutinas de recepcion y manejo de datos
  void packetMng(dataPacketMT2);
  bool frameMng(std::vector<unsigned char>);
  
public:
  
  XSensMTi700Driver();
  ~XSensMTi700Driver();
  
  // Operaciones a alto nivel
  bool connectToDevice();
  void disconnectDevice();
  void configureDevice();
  bool getData();
  
  // Retorno de estructura de datos
  GPSINSInfo getInfo();
};

#endif	/* XSENSMTI700DRIVER_H */

/**
 * @}
 */
