/* 
 * File:   DrivingConnectionManager.h
 * Author: Carlos Amores
 *
 * Created on 15 de junio de 2014, 16:10
 */

#ifndef DRIVINGCONNECTIONMANAGER_H
#define	DRIVINGCONNECTIONMANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* DrivingConnectionManager */

#include "ros/ros.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <arpa/inet.h>

typedef struct {
    short steering;
    short thottle;
    short brake;
    bool parkingBrake;
    unsigned short gear;
    unsigned short speed;
    short motorRPM;
    short motorTemperature;
    bool lights;
    bool blinkerLeft;
    bool blinkerRight;
    bool dipss;
    bool dipsr;
    bool dipsp;
    bool klaxon;
}DrivingInfo;

class DrivingConnectionManager{
private:
    // Socket
    int socketDescriptor;
public:
    // Constructor
    DrivingConnectionManager();
    // Gestion del vehiculo
    bool connectVehicle();
    bool setParam(short idParam, float value);
    short getParam(short idParam);
    DrivingInfo reqBasicVehicleInfo();
    DrivingInfo reqFullVehicleInfo();
    bool disconnectVehicle();
    // Getter y setter necesarios
    int getSocketDescriptor();

};

