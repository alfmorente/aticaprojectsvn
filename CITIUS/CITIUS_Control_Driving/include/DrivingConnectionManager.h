
/** 
 * @file  DrivingConnectionManager.h
 * @brief Declara el tipo de la clase "DrivingConnectionManager"
 * - La clase implementa la comunicacion con el modulo de conduccion del 
 * subsistema payload de conduccion.
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#ifndef DRIVINGCONNECTIONMANAGER_H
#define	DRIVINGCONNECTIONMANAGER_H

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

#endif	/* DrivingConnectionManager */


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
    // Modificadores privados
    void setVehicleInfo(short id_device, short value);
    short getDriveAlarms();
    short getSteeringAlarms();
    
    
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
    DrivingInfo getVehicleInfo(bool full);
    int getSocketDescriptor();
    short getCountCriticalMessages();
    void setCountCriticalMessages(short cont);
    
    // Metodos auxiliares
    bool isCriticalInstruction(short element);
    
    // Tratamiento de la cola de mensajes criticos
    void addToQueue(FrameDriving frame);
    RtxStruct informResponse(bool,short);
    
    // Tratamiento de atributos de alarmas
    void setAlarmsInfo(short element, short value);
    
};

