
/** 
 * @file  DrivingConnectionManager.h
 * @brief Declara el tipo de la clase "DrivingConnectionManager"
 * - La clase implementa la comunicación con el módulo de conducción del 
 * subsistema payload de conducción
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup DrivingDriver
 * @{
 */

#ifndef DRIVINGCONNECTIONMANAGER_H
#define	DRIVINGCONNECTIONMANAGER_H

#include <vector>
#include "ros/ros.h"
#include "constant.h"
#include "SocketDriver.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <fstream>

using namespace std;

/**
 * \class DrivingConnectionManager
 * \brief Clase que representa al driver de comunicación con el módulo de
 * conducción del vehículo
 */
class DrivingConnectionManager: public SocketDriver {
private:
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
  void setVehicleInfo(DeviceID element, short value);
  void setAlarmsInfo(DeviceID element, short value);
  // Metodos de gestion privados
  RtxStruct informResponse(bool ack, short id_instruction);
public:
  // Constructor
  DrivingConnectionManager();
  // Destructor
  ~DrivingConnectionManager();
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
  bool isCriticalInstruction(DeviceID element);
  bool isMTCommand(DeviceID element);
  // Tratamiento de la cola de mensajes criticos
  void addToQueue(FrameDriving frame);
};

#endif	/* DrivingConnectionManager */

/**
 * @}
 */