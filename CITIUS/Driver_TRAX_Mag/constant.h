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

typedef struct {
    char *len;
    char *payload;
    char *cs;
} TraxMsg;

typedef struct {
    unsigned char idData;
    unsigned char Value;
} PayloadData;

void sendToDevice(TraxMsg);
void waitForAck(unsigned char);
unsigned char calcChecksum(TraxMsg);
bool isCheckSumOK(TraxMsg);
TraxMsg kGetData();


float hexa2float(unsigned char * );
double hexa2double(unsigned char * );
int hexa2int(unsigned char * );



struct termios newtio, oldtio;
int canal;

