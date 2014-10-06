
/** 
 * @file  TraxAHRSModuleDriver.h
 * @brief Declara el tipo de la clase "TraxAHRSModuleDriver"
 * - La clase implementa la comunicacion con el dispositivo magnetometro TRAX
 * AHRS Module encargado de obtener la orientacion del vehiculo
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef TRAXAHRSMODULEDRIVER_H
#define	TRAXAHRSMODULEDRIVER_H

// ID Frame
#define IDFRAME_KGETMODINFO 0x01
#define IDFRAME_KGETMODINFORESP 0x02
#define IDFRAME_KSETDATACOMPONENTS 0x03
#define IDFRAME_KGETDATA 0x04
#define IDFRAME_KGETDATARESP 0x05

// ID MEASURE

#define IDMEASURE_HEADING 0x05
#define IDMEASURE_PITCH 0x18
#define IDMEASURE_ROLL 0x19
#define IDMEASURE_HEADING_STATUS 0x4F
#define IDMEASURE_ACCX 0x15
#define IDMEASURE_ACCY 0x16
#define IDMEASURE_ACCZ 0x17
#define IDMEASURE_GYRX 0x4A
#define IDMEASURE_GYRY 0x4B
#define IDMEASURE_GYRZ 0x4C

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

typedef struct {
  char idFrame; /// Identificador de la trama
  std::vector<char> payload; /// Valor del campo payload de la trama en crudo
} PacketFrame;

typedef struct {
  std::vector<char> byteCount; /// Valor del campo byteCount de la trama en crudo
  PacketFrame packFrame; /// Trama semidesempaquetada
  std::vector<char> crc; /// Valor del CRC16 en crudo
  bool checked; /// Indicador de tratamiento de la trama finalizado
} TraxMsg;

typedef struct {
  char heading_status; /// Valor del estado de la orientacion obtenida
  float heading; /// Valor de la orientacion en yaw (Z)
  float pitch; /// Valor de la orientacion en pitch (X)
  float roll; /// Valor de la orientacion en roll (Y)
  float accX; /// Aceleracion longitudinal componente X
  float accY; /// Aceleracion longitudinal componente Y
  float accZ; /// Aceleracion longitudinal componente Z
  float gyrX; /// Velocidad rotacional componente X
  float gyrY; /// Velocidad rotacional componente Y
  float gyrZ; /// Velocidad rotacional componente Z
} TraxMeasurement;

using namespace std;

#endif	/* TRAXAHRSMODULEDRIVER_H */



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


