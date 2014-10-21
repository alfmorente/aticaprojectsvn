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
  void sendThrottleInfo(int socketDescriptor);
  void sendBrakeInfo(int socketDescriptor);
  void sendHandbrakeInfo(int socketDescriptor);
  void sendSteeringInfo(int socketDescriptor);
  void sendGearInfo(int socketDescriptor);
  void sendSteeringAlarmsInfo(int socketDescriptor);
  void sendDriveAlarmsInfo(int socketDescriptor);
  void sendBlinkerLeftInfo(int socketDescriptor);
  void sendBlinkerRightInfo(int socketDescriptor);
  void sendEmergencyBlinkerInfo(int socketDescriptor);
  void sendDipssInfo(int socketDescriptor);
  void sendDipsrInfo(int socketDescriptor);
  void sendDipspInfo(int socketDescriptor);
  void sendKlaxonInfo(int socketDescriptor);
  void sendMTThrottleInfo(int socketDescriptor);
  void sendMTBrakeInfo(int socketDescriptor);
  void sendMTHandbrakeInfo(int socketDescriptor);
  void sendMTSteeringInfo(int socketDescriptor);
  void sendMTGearInfo(int socketDescriptor);
  void sendMTBlinkersInfo(int socketDescriptor);
  void sendMTLightsInfo(int socketDescriptor);
  void sendMotorRPMInfo(int socketDescriptor);
  void sendMotorTemperatureInfo(int socketDescriptor);
  void sendCruissingSpeedInfo(int socketDescriptor);
};

#endif	/* MESSAGEDISPATCHER_H */

