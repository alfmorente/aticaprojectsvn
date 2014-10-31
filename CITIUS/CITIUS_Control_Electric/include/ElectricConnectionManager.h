
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
  void setVehicleInfo(DeviceID id_device, short value);
  short getSupplyAlarms();
  // Metodos de gestion privados
  RtxStruct informResponse(bool ack, short id_instruction);
  bool isCriticalInstruction(DeviceID element);
public:
  // Constructor
  ElectricConnectionManager();
  // Destructor
  ~ElectricConnectionManager();
  // Gestion del vehiculo
  string getValueFromConfig(string parameter);
  bool connectVehicle();
  bool disconnectVehicle();
  // Mensajeria con vehiculo
  void sendToVehicle(FrameDriving);
  bool checkForVehicleMessages();
  void reqElectricInfo();
  short waitForSwitcherPosition();
  void setTurnOn();
  void setTurnOff();
  // Getter y setter necesarios
  bool getTurnOffFlag();
  ElectricInfo getVehicleInfo();
  short getCountCriticalMessages();
  void setCountCriticalMessages(short count);
  void setSwitcherStruct(bool position);
  SwitcherStruct getSwitcherStruct();
};

#endif	/* ElectricConnectionManager */

/**
 * @}
 */