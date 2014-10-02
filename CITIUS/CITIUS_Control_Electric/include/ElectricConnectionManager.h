/* 
 * File:   ElectricConnectionManager.h
 * Author: Carlos Amores
 *
 * Created on 15 de junio de 2014, 16:10
 */

#ifndef ELECTRICCONNECTIONMANAGER_H
#define	ELECTRICCONNECTIONMANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* ElectricConnectionManager */

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

class ElectricConnectionManager{
private:
    
    // Socket
    int socketDescriptor;
    
    // Contador para la cola de mensajes (Integridad)
    short countMsg;
    
    // Manejador de la cola de mensajes (Integridad)
    vector<FrameDriving> messageQueue;
    
    // Informacion actualizable del vehiculo
    ElectricInfo electricInfo;
    
    // Solicitud de apagado del vehiculo
    bool turnOff;
    
    // Posicion del conmutador local/teleoperado
    SwitcherStruct swPosition;
    
    // Alarmas del subsistema electrico
    short supplyAlarms;
    
public:
    // Constructor
    ElectricConnectionManager();
    
    // Gestion del vehiculo
    bool connectVehicle();
    bool disconnectVehicle();
    
    // Mensajeria con vehiculo
    void sendToVehicle(FrameDriving);
    bool checkForVehicleMessages();
    void reqElectricInfo();
    void setTurnOff();
    
    // Getter y setter necesarios
    int getSocketDescriptor();
    short getCountCriticalMessages();
    void setCountCriticalMessages(short);
    ElectricInfo getVehicleInfo();
    void setVehicleInfo(short, short);
    bool getTurnOffFlag();
    SwitcherStruct getSwitcherStruct();
    void setSwitcherStruct(bool);
    short getSupplyAlarms();
    
    // Metodos auxiliares
    bool isCriticalInstruction(short element);
    
    // Tratamiento de la cola de mensajes criticos
    void addToQueue(FrameDriving frame);
    RtxStruct informResponse(bool,short);

};
