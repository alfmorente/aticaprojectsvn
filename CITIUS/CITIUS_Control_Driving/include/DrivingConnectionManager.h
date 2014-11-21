
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
  short countMsg;
  vector<FrameDriving> messageQueue;
  DrivingInfo vehicleInfo;
  AlarmsStruct alarms;
  short steeringAlarms;
  void setVehicleInfo(DeviceID element, short value);
  void setAlarmsInfo(DeviceID element, short value);
  RtxStruct informResponse(bool ack, short id_instruction);
public:
  DrivingConnectionManager();
  ~DrivingConnectionManager();
  void sendToVehicle(FrameDriving frame);
  void reqVehicleInfo(bool full);
  bool checkForVehicleMessages();
  DrivingInfo getVehicleInfo(bool full);
  int getSocketDescriptor();
  short getCountCriticalMessages();
  void setCountCriticalMessages(short cont);
  AlarmsStruct getAlarmsStruct();
  void setAlarmsStruct(bool flag);
  bool isCriticalInstruction(DeviceID element);
  bool isMTCommand(DeviceID element);
  void addToQueue(FrameDriving frame);
};

#endif	/* DrivingConnectionManager */

/**
 * @}
 */