/* 
 * File:   constant.h
 * Author: atica
 *
 * Created on 24 de julio de 2014, 17:27
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

#include <cstdlib>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sstream>

#define IP_CAMERA "192.168.24.120"
#define PORT_CAMERA 80

#define AUTH_CAM_USER "root"
#define AUTH_CAM_PASS "usv"

#define PTZ_ROUTE "/axis-cgi/com/ptz.cgi?"

#define ORDER_ZOOM 0
#define ORDER_PAN 1
#define ORDER_TILT 2

int socketDescriptor;
struct hostent *he;
struct sockaddr_in server;

typedef struct{
    bool state;
    float zoom;
    float pan;
    float tilt;
}LensPosition;

bool sentSetToDevice(short order, float value);
LensPosition getPosition();

float extractZoom(char[]);
float extractTilt(char[]);
float extractPan(char[]);