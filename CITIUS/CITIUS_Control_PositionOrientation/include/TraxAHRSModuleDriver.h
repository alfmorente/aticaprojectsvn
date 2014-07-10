/* 
 * File:   TraxAHRSModuleDriver.h
 * Author: Carlos Amores
 *
 * Created on 10 de julio de 2014, 21:04
 */

#ifndef TRAXAHRSMODULEDRIVER_H
#define	TRAXAHRSMODULEDRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* TRAXAHRSMODULEDRIVER_H */

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
  char idFrame;
  std::vector<char> payload;
} PacketFrame;

typedef struct {
  std::vector<char> byteCount;
  PacketFrame packFrame;
  std::vector<char> crc;
  bool checked;
} TraxMsg;

typedef struct {
  char heading_status;
  float heading;
  float pitch;
  float roll;
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
} TraxMeasurement;

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


