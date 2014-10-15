
/** 
 * @file  RosNode_Electric.h
 * @brief Declara el tipo de la clase "RosNode_Electric"
 * - La clase implementa la gestión del nodo de conducción (Electric) del 
 * Subsistema de control de UGV
 * @author Carlos Amores
 * @date 2013, 2014
 * @addtogroup Control Subsistema de Control
 * @{
 */

#ifndef ROSNODE_ELECTRIC_H
#define	ROSNODE_ELECTRIC_H

#include <time.h>
#include "RosNode.h"
#include "ElectricConnectionManager.h"
#include "CITIUS_Control_Electric/msg_electricInfo.h"
#include "CITIUS_Control_Electric/msg_command.h"
#include "CITIUS_Control_Electric/msg_switcher.h"
#include "CITIUS_Control_Electric/srv_vehicleStatus.h"
#include <deque>
#include <cstdlib>

using namespace std;

/**
 * /class RosNode_Driving
 * /brief Clase que representa al nodo ROS que gestiona la comunicación con el 
 * módulo de alimentación del vehículo
*/
class RosNode_Electric: public RosNode {
private:
  // Estado del nodo
  //short emNodeStatus;
  // Publicador de informacion de camara
  ros::Publisher pubElectricInfo;
  ros::Publisher pubCommand;
  ros::Publisher pubSwitcher;
  // Cliente de estado de vehiculo
  ros::ServiceClient clientVehicleStatus;
  // Driver de la cámara
  ElectricConnectionManager *dElectric;
public:
  // Constructor
  RosNode_Electric();

  // Inicializador de artefactos ROS
  void initROS();

  // Getter and Setter necesarios
  ros::ServiceClient getClientVehicleStatus();
  //short getEMNodeStatus();
  //void setEMNodeStatus(short newEMNodeStatus);
  ElectricConnectionManager *getDriverMng();

  // Publicar informacion de vehiculo
  void publishElectricInfo(ElectricInfo info);
  // Publicar informacion de conmutador Local/Teleoperado
  void publishSwitcherInfo(short position);
  // Publicar consignas ON/OFF de actuadores
  void publishSetupCommands(bool on);
};

#endif	/* ROSNODE_ELECTRIC_H */

/**
 * @}
 */
