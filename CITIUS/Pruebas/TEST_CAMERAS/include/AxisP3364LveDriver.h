/* 
 * File:   AxisP3364LveDriver.h
 * Author: atica
 *
 * Created on 30 de julio de 2014, 10:41
 */

#ifndef AXISP3364LVEDRIVER_H
#define	AXISP3364LVEDRIVER_H

#ifdef	__cplusplus
extern "C" {
#endif


#ifdef	__cplusplus
}
#endif

#endif	/* AXISP3364LVEDRIVER_H */

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

typedef struct{
    bool state;
    float zoom;
    float pan;
    float tilt;
}LensPosition;

using namespace std;

class AxisP3364LveDriver{
    
public:
    
    AxisP3364LveDriver();
    ~AxisP3364LveDriver();
    bool sentSetToDevice(short order, float value);
    LensPosition getPosition();

private:

    int socketDescriptor;
    struct hostent *he;
    struct sockaddr_in server;

    float extractZoom(char[]);
    float extractTilt(char[]);
    float extractPan(char[]);
    
};