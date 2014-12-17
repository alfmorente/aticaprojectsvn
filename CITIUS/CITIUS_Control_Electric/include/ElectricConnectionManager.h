
/** 
 * @file  ElectricConnectionManager.h
 * @brief Declara el tipo de la clase "ElectricConnectionManager"
 * - La clase implementa la comunicación con el módulo electrico del 
 * subsistema payload de conducción.
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup ElectricDriver
 * @{
 */

#ifndef ELECTRICCONNECTIONMANAGER_H
#define	ELECTRICCONNECTIONMANAGER_H

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
 * \class ElectricConnectionManager
 * \brief Clase que representa al driver de comunicación con el módulo de
 * alimentación del vehículo
 */
class ElectricConnectionManager:public SocketDriver {
private:
  short countMsg;
  vector<FrameDriving> messageQueue;
  ElectricInfo electricInfo;
  SystemSupplies systemSupplies;
  bool turnOff;
  SupplyAlarmsStruct alarms;
  void setVehicleInfo(DeviceID id_device, short value);
  RtxStruct informResponse(bool ack, short id_instruction);
  bool isCriticalInstruction(DeviceID element);
public:
  ElectricConnectionManager();
  ~ElectricConnectionManager();
  void sendToVehicle(FrameDriving);
  bool checkForVehicleMessages();
  void reqElectricInfo();
  void setTurnOn();
  void setTurnOff();
  bool getTurnOffFlag();
  ElectricInfo getVehicleInfo();
  short getCountCriticalMessages();
  void setCountCriticalMessages(short count);
  void setSupplyAlarmsStruct(bool flag);
  SupplyAlarmsStruct getSupplyAlarmsStruct();
};

#endif	/* ElectricConnectionManager */

/**
 * @}
 */