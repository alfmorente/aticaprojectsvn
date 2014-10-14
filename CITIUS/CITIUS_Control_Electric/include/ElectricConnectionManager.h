
/** 
 * @file  ElectricConnectionManager.h
 * @brief Declara el tipo de la clase "ElectricConnectionManager"
 * - La clase implementa la comunicación con el módulo electrico del 
 * subsistema payload de conducción.
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef ELECTRICCONNECTIONMANAGER_H
#define	ELECTRICCONNECTIONMANAGER_H

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
 * /class ElectricConnectionManager
 * /brief Clase que representa al driver de comunicación con el módulo de
 * alimentación del vehículo
*/
class ElectricConnectionManager {
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
  //Consultores/modificadores privados
  void setVehicleInfo(short, short);
  short getSupplyAlarms();
  // Metodos de gestion privados
  RtxStruct informResponse(bool, short);
  bool isCriticalInstruction(short element);

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
  bool getTurnOffFlag();
  ElectricInfo getVehicleInfo();
  short getCountCriticalMessages();
  void setCountCriticalMessages(short);
  void setSwitcherStruct(bool);
  SwitcherStruct getSwitcherStruct();

  // Metodos auxiliares
  

  // Tratamiento de la cola de mensajes criticos
  void addToQueue(FrameDriving frame);
  
};

#endif	/* ElectricConnectionManager */

/**
 * @}
 */