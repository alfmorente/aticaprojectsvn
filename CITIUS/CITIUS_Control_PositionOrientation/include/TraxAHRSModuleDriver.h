
/** 
 * @file  TraxAHRSModuleDriver.h
 * @brief Declara el tipo de la clase "TraxAHRSModuleDriver"
 * - La clase implementa la comunicación con el dispositivo magnetómetro TRAX
 * AHRS Module encargado de obtener la orientación del vehículo
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef TRAXAHRSMODULEDRIVER_H
#define	TRAXAHRSMODULEDRIVER_H

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>
#include <complex>
#include "crc16calc.h"
#include "conversionTypes.h"
#include <vector>
#include "constant.h"

using namespace std;

/**
 * /class TraxAHRSModuleDriver
 * /brief Clase que representa al driver de comunicación con el dispositivo
 * magnetómetro Trax AHRS Module
*/
class TraxAHRSModuleDriver {
private:
  // Datos recibidos
  TraxMeasurement oriInfo;

  // Puerto serie
  struct termios newtio, oldtio;
  int canal;

  // Operaciones a bajo nivel
  TraxMsg kGetModInfo();
  TraxMsg kGetData();
  TraxMsg kSetDataComponents();

  void sendToDevice(TraxMsg);
  void rcvResponse();
  TraxMsg mngPacket(std::vector< char>);
  TraxMeasurement unpackPayload(std::vector<char>);

public:

  TraxAHRSModuleDriver();
  ~TraxAHRSModuleDriver();

  // Operaciones a alto nivel
  bool connectToDevice();
  void disconnectDevice();
  void configureDevice();
  bool getData();
  
  // Retorno de estructura de datos
  TraxMeasurement getInfo();
};

#endif	/* TRAXAHRSMODULEDRIVER_H */

/**
 * @}
 */

