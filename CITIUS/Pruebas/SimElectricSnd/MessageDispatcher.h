/* 
 * File:   MessageDispatcher.h
 * Author: atica
 *
 * Created on 21 de octubre de 2014, 16:50
 */

#ifndef MESSAGEDISPATCHER_H
#define	MESSAGEDISPATCHER_H

using namespace std;

#include <stdio.h>          
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>

class MessageDispatcher{
public:
  MessageDispatcher();
  ~MessageDispatcher();
  void sendBatteryLevelMsg(int socketDescriptor);
  void sendBatteryVoltageMsg(int socketDescriptor);
  void sendBatteryCurrentMsg(int socketDescriptor);
  void sendBatteryTemperatureMsg(int socketDescriptor);
  void sendSupplyCheckMsg(int socketDescriptor);
  void sendTurnOffMsg(int socketDescriptor);
  void sendSupply5Msg(int socketDescriptor);
  void sendSupply12Msg(int socketDescriptor);
  void sendSupply24DriveMsg(int socketDescriptor);
  void sendSupply24OCCMsg(int socketDescriptor);
  void sendControlSystemSupplyMsg(int socketDescriptor);
  void sendControlSystemSupply5Msg(int socketDescriptor);
  void sendControlSystemSupply12Msg(int socketDescriptor);
  void sendControlSystemSupply24Msg(int socketDescriptor);
  void sendControlSystemSupply48Msg(int socketDescriptor);
  void sendDriveSystemSupplyMsg(int socketDescriptor);
  void sendDriveSystemSupply5Msg(int socketDescriptor);
  void sendDriveSystemSupply12Msg(int socketDescriptor);
  void sendDriveSystemSupply24Msg(int socketDescriptor);
  void sendDriveSystemSupply48Msg(int socketDescriptor);
  void sendCommSystemSupplyMsg(int socketDescriptor);
  void sendCommSystemSupply5Msg(int socketDescriptor);
  void sendCommSystemSupply12Msg(int socketDescriptor);
  void sendCommSystemSupply24Msg(int socketDescriptor);
  void sendCommSystemSupply48Msg(int socketDescriptor);
  void sendObservationSystemSupplyMsg(int socketDescriptor);
  void sendObservationSystemSupply5Msg(int socketDescriptor);
  void sendObservationSystemSupply12Msg(int socketDescriptor);
  void sendObservationSystemSupply24Msg(int socketDescriptor);
  void sendObservationSystemSupply48Msg(int socketDescriptor);
  void sendSupplyAlarmsMsg(int socketDescriptor);
  void sendOperationModeSwitchMsg(int socketDescriptor);

};

#endif	/* MESSAGEDISPATCHER_H */

