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

class DrivingConnectionManager{
private:
    // Socket
    int socketDescriptor;
public:
    // Constructor
    DrivingConnectionManager();
    // Gestion del vehiculo
    bool connectVehicle();
    void setParam(short idParam, float value);
    void getParam(short idParam);
    void reqVehicleInfo();
    bool disconnectVehicle();
    // Getter y setter necesarios
    void setSocketDescriptor(int newSocketDescriptor);
    int getSocketDescriptor();

};

