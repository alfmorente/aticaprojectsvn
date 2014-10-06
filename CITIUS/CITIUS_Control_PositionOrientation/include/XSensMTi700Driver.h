
/** 
 * @file  XSensMTi700Driver.h
 * @brief Declara el tipo de la clase "XSensMTi700Driver"
 * - La clase implementa la comunicacion con el dispositivo magnetometro X-Sens 
 * MTi-G 700 encargado de obtener la posicion y orientacion del vehiculo
 * @author: Carlos Amores
 * @date: 2013, 2014
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

#define FREC_REQ_DATA 0x0A

// Informacion de GPS/INS
typedef struct{
  // Estado
  short positionStatus; /// Valor del estado de la posicion
  short orientationStatus; /// Valor del estado de la orientacion
  // Posicion
  double latitude; /// Valor de posicion en latitud
  double longitude; /// Valor de la posicion en longitud
  float altitude; /// Valor de la posicion en altitud
  // Orientacion
  float roll; /// Valor de la orientacion en roll (Y)
  float pitch; /// Valor de la orientacion en pitch (X)
  float yaw; /// Valor de la orientacion en yaw (Z)
  // Velocidad longitudinal
  float velX; /// Valor de la velocidad longitudinal componente X
  float velY; /// Valor de la velocidad longitudinal componente Y
  float velZ; /// Valor de la velocidad longitudinal componente Z
  // Acc longitudinal
  float accX; /// Valor de la aceleracion longitudinal componente X
  float accY; /// Valor de la aceleracion longitudinal componente Y
  float accZ; /// Valor de la aceleracion longitudinal componente Z
  // Acc rotacional
  float rateX; /// Valor de la aceleracion rotacional componente X
  float rateY; /// Valor de la aceleracion rotacional componente Y
  float rateZ; /// Valor de la aceleracion rotacional componente Z
} GPSINSInfo;

// Mensaje XSens
typedef struct {
    unsigned char pre; /// Valor en crudo del campo PRE de la trama
    unsigned char bid; /// Valor en crudo del campo BID de la trama
    unsigned char mid; /// Valor en crudo del campo MID de la trama
    unsigned char len; /// Valor en crudo del campo LEN de la trama
    std::vector<unsigned char> data; /// Datos en crudo de la trama
    unsigned char cs; /// Valor en crudo del campo CS de la trama
} XsensMsg;

// Paquete de datos MTData2 (XSens Low level protocol)
typedef struct {
  unsigned char idGroup; /// Valor del identificador del grupo de datos (MTData2)
  unsigned char idSignal; /// Valor del identificador de la señal (MTData2)
  unsigned char len; /// Valor del campo longitud (MTData2)
  std::vector<unsigned char> data; /// Valor de los datos en crudo (MTData2)
} dataPacketMT2;

using namespace std;

#endif	/* XSENSMTI700DRIVER_H */



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
  
  float hexa2float(std::vector<unsigned char>);
  double hexa2double(std::vector<unsigned char>);
  int hexa2int(std::vector<unsigned char>);
  
  // Rutina de recepcion de datos (thread)
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

