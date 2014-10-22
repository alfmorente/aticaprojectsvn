/* 
 * File:   SimuladorPayloadConduccion.h
 * Author: atica
 *
 * Created on 19 de junio de 2014, 10:10
 */

#ifndef SIMULADORPAYLOADCONDUCCION_H
#define	SIMULADORPAYLOADCONDUCCION_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* SIMULADORPAYLOADCONDUCCION_H */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <iostream>

#define IP_PAYLOAD_CONDUCCION_DRIVING "127.0.0.1"
#define PORT_SERVER 3500

typedef struct{
    short instruction;
    short id_instruccion;
    short element;
    short value;
}FrameDriving;

