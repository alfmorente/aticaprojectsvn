/* 
 * File:   constant.h
 * Author: atica
 *
 * Created on 1 de julio de 2014, 17:56
 */

#ifndef CONSTANT_H
#define	CONSTANT_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* CONSTANT_H */

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
#define COMMAND_MID_MTDATA2 0x36
#define COMMAND_LEN_0 0x00

typedef struct {
    unsigned char pre;
    unsigned char bid;
    unsigned char mid;
    unsigned char len;
    unsigned char *data;
    unsigned char cs;
} xsensMsg;

typedef struct {
  bool temperature;
  bool calibrated_data;
  bool orientation;
  bool auxiliary_data;
  bool position;
  bool velocity;
  bool status;
  bool raw_gps;
  bool raw_ins;
} outPutMode;

typedef struct {
  unsigned short idGroup;
  unsigned short idSignal;
  unsigned char len;
  unsigned char *data;
} dataPacketMT2;


void sendToDevice(xsensMsg);
void waitForAck(unsigned char);
unsigned char calcChecksum(xsensMsg);
bool isCheckSumOK(xsensMsg);
xsensMsg reqDeviceID();
xsensMsg goToMeasurement();
xsensMsg goToConfig();
xsensMsg setOutPutMode();
xsensMsg setOutPutSettings();
void streamDataMng();
void packetMng(dataPacketMT2);

float hexa2float(unsigned char * );
int hexa2int(unsigned char * );
float parseFixPointFormat(unsigned char *);



struct termios newtio, oldtio;
int canal;

