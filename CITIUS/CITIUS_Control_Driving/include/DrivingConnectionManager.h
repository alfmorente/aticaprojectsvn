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
#include <vector>
#include "ros/ros.h"
#include "constant.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <arpa/inet.h>

using namespace std;

class DrivingConnectionManager{
private:
    
    // Socket
    int socketDescriptor;
    
    // Contador para la cola de mensajes (Integridad)
    short countMsg;
    
    // Manejador de la cola de mensajes (Integridad)
    vector<FrameDriving> messageQueue;
    
    // Informacion actualizable del vehiculo
    DrivingInfo vehicleInfo;
    
    // Alarmas de conduccion
    short driveAlarms;
    // Alarmas de direccion
    short steeringAlarms;
    
public:
    // Constructor
    DrivingConnectionManager();
    
    // Gestion del vehiculo
    bool connectVehicle();
    bool disconnectVehicle();
    
    // Mensajeria con vehiculo
    void sendToVehicle(FrameDriving frame);
    void reqVehicleInfo(bool full);
    bool checkForVehicleMessages();
    
    // Getter y setter necesarios
    int getSocketDescriptor();
    short getCountCriticalMessages();
    void setCountCriticalMessages(short cont);
    DrivingInfo getVehicleInfo(bool full);
    void setVehicleInfo(short id_device, short value);
    short getDriveAlarms();
    short getSteeringAlarms();
    
    // Metodos auxiliares
    bool isCriticalInstruction(short element);
    
    // Tratamiento de la cola de mensajes criticos
    void addToQueue(FrameDriving frame);
    RtxStruct informResponse(bool,short);
    
    // Tratamiento de atributos de alarmas
    void setAlarmsInfo(short element, short value);
    
};

