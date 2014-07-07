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
#include <stdint.h>

typedef uint16_t bit_order_16(uint16_t value);
typedef uint8_t bit_order_8(uint8_t value);

uint16_t straight_16(uint16_t);

uint8_t straight_8(uint8_t);

uint16_t crc16(uint8_t  *message, int nBytes,bit_order_8 , bit_order_16 ,uint16_t , uint16_t );

uint16_t crc16ccitt_xmodem(uint8_t  *, int );



void sendToDevice();
void read();


float hexa2float(unsigned char * );
double hexa2double(unsigned char * );
int hexa2int(unsigned char * );
char *shortToHexa(short);



struct termios newtio, oldtio;
int canal;

