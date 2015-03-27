
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
#include "constantes.h"
#include "SocketDriver.h"
#include "Timer.hpp"
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
  short byteCount;
  FrameDriving frameRcv;
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
  void reqVehicleInfo();
  bool checkForVehicleMessages();
  DrivingInfo getVehicleInfo(bool full);
  int getSocketDescriptor();
  void setCountCriticalMessages(short cont);
  AlarmsStruct getAlarmsStruct();
  void setAlarmsStruct(bool flag);
  bool isCriticalInstruction(DeviceID element);
  bool isMTCommand(DeviceID element);
  void setCommand(FrameDriving command);
  void setAlarmsInfo(short id_alarm);
  void checkMessageTimeout();
  DrivingInfo getLastCommandsInfo();
  bool checkSwitcher();
  int getSwitcher();
  void sendReset();
};

#endif	/* DrivingConnectionManager */

/**
 * @}
 */