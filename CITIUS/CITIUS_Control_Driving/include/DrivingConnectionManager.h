
/** 
 * @file  DrivingConnectionManager.h
 * @brief Declara el tipo de la clase "DrivingConnectionManager"
 * - La clase implementa la comunicación con el módulo de conducción del 
 * subsistema payload de conducción
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
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

using namespace std;

/**
 * /class DrivingConnectionManager
 * /brief Clase que representa al driver de comunicación con el módulo de
 * conducción del vehículo
*/
class DrivingConnectionManager {
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
  void setAlarmsInfo(short element, short value);
  // Metodos de gestion privados
  RtxStruct informResponse(bool, short);


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
    

};

#endif	/* DrivingConnectionManager */

/**
 * @}
 */