/* 
 * File:   XSensMTi700Driver.h
 * Author: Carlos Amores
 *
 * Created on 4 de julio de 2014, 19:01
 */

#ifndef XSENSMTI700DRIVER_H
#define	XSENSMTI700DRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* XSENSMTI700DRIVER_H */

#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>
#include <complex>
#include <pthread.h>

#define COMMAND_PRE 0xFA
#define COMMAND_BID 0xFF
#define COMMAND_MID_GOTOCONFIG 0x30
#define COMMAND_MID_GOTOMEASUREMENT 0x10
#define COMMAND_MID_SETOUTPUTMODE 0xD0
#define COMMAND_MID_SETOUTPUTSETTINGS 0xD2
#define COMMAND_MID_REQDID 0x00
#define COMMAND_MID_SETOUTPUTSKIPFACTOR 0xD4
#define COMMAND_MID_SETPERIOD 0x04
#define COMMAND_MID_SETOUTPUTCONFIGURATION 0xC0
#define COMMAND_MID_MTDATA2 0x36
#define COMMAND_LEN_0 0x00

#define FREC_REQ_DATA 0x19

// Informacion de GPS/INS
typedef struct{
  // Estado
  short positionStatus;
  short orientationStatus;
  // Posicion
  double latitude;
  double longitude;
  float altitude;
  // Orientacion
  float roll;
  float pitch;
  float yaw;
  // Velocidad longitudinal
  float velX;
  float velY;
  float velZ;
  // Acc longitudinal
  float accX;
  float accY;
  float accZ;
  // Acc rotacional
  float rateX;
  float rateY;
  float rateZ;
} GPSINSInfo;

// Mensaje XSens
typedef struct {
    unsigned char pre;
    unsigned char bid;
    unsigned char mid;
    unsigned char len;
    unsigned char *data;
    unsigned char cs;
} XsensMsg;

// Paquete de datos MTData2 (XSens Low level protocol)
typedef struct {
  unsigned char idGroup;
  unsigned char idSignal;
  unsigned char len;
  unsigned char *data;
} dataPacketMT2;

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
  bool isCheckSumOK(unsigned char*, unsigned char);
  
  XsensMsg goToConfig();
  XsensMsg goToMeasurement();
  XsensMsg setOutPutConfiguration();
  
  float hexa2float(unsigned char *);
  double hexa2double(unsigned char *);
  int hexa2int(unsigned char *);
  
  // Rutina de recepcion de datos (thread)
  void packetMng(dataPacketMT2);
  
public:
  
  XSensMTi700Driver();
  
  // Operaciones a alto nivel
  bool connectToDevice();
  void disconnectDevice();
  void configureDevice();
  bool getData();
  
  // Retorno de estructura de datos
  GPSINSInfo getInfo();
  bool frameMng(unsigned char*, unsigned char);
};

