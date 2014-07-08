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

#define IDFRAME_KGETMODINFO 0x01
#define IDFRAME_KSETDATACOMPONENTS 0x03
#define IDFRAME_KGETDATA 0x04
#define IDFRAME_KGETDATARESP 0x05


#include <iostream>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h> 
#include <string.h>
#include <complex>
#include <stdint.h>

typedef struct{
    char idValue;
    char *value;
}PacketData;

typedef struct{
    char idCount;
    PacketData *packetData;
}Payload;

typedef struct{
    char idFrame;
    Payload payload;
}PacketFrame;

typedef struct{
    char *byteCount;
    PacketFrame packFrame;
    char *crc;
}TraxMsg;





typedef uint16_t bit_order_16(uint16_t value);
typedef uint8_t bit_order_8(uint8_t value);

uint16_t straight_16(uint16_t);

uint8_t straight_8(uint8_t);

uint16_t crc16(uint8_t  *message, int nBytes,bit_order_8 , bit_order_16 ,uint16_t , uint16_t );

uint16_t crc16ccitt_xmodem(uint8_t  *, int );


TraxMsg kGetModInfo();
TraxMsg kGetData();
TraxMsg kSetDataComponents();


void sendToDevice(TraxMsg);
void rcvResponse(char);


float hexa2float(unsigned char * );
double hexa2double(unsigned char * );
int hexa2int(unsigned char * );
short hexa2short(char[2]);
char *shortToHexa(short);



struct termios newtio, oldtio;
int canal;

