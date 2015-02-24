
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
#include "Timer.h"
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
  Timer *messagesTimeout;
  UpdateRegs updateRegs;
  DrivingInfo vehicleInfo;
  DrivingInfo lastCommandsSent;
  AlarmsStruct alarms;
  short steeringAlarms;
  SwitcherStruct swPosition;
  void setVehicleInfo(DeviceID element, short value);
  void setAlarmsInfo(DeviceID element, short value);
  bool isCommandInQueue(short idDevice);
  void manageACK(short element, short id_instruccion);
  void manageNACK(short id_instruccion);
  bool isThereCommandReady(short element);
  void sendCommandFromUpdateRegs(short element);
  void fillLastCommand(short element, short value);
public:
  DrivingConnectionManager();
  ~DrivingConnectionManager();
  void sendToVehicle(FrameDriving frame);
  void reqVehicleInfo(bool full);
  bool checkForVehicleMessages();
  DrivingInfo getVehicleInfo(bool full);
  int getSocketDescriptor();
  void setCountCriticalMessages(short cont);
  AlarmsStruct getAlarmsStruct();
  void setAlarmsStruct(bool flag);
  bool isCriticalInstruction(DeviceID element);
  bool isMTCommand(DeviceID element);
  void setSwitcherStruct(bool position);
  SwitcherStruct getSwitcherStruct();
  short waitForSwitcherPosition();
  void setCommand(FrameDriving command);
  void setAlarmsInfo(short id_alarm);
  void checkMessageTimeout();
  DrivingInfo getLastCommandsInfo();
};

#endif	/* DrivingConnectionManager */

/**
 * @}
 */