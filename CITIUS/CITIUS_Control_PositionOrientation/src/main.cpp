/** 
 * @file  main.cpp
 * @brief Funcion principal del nodo PositionOrientation del subsistema de 
 * control
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNose_PositionOrientation.h"

/**
 * Método principal del nodo. Inicializa modulos ROS y lanza el intercambio de 
 * mensajes con los dispositivos
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "RosNode_Control_Position_Orientation");
  RosNode_PositionOrientation *nodePosOri = new RosNode_PositionOrientation();

  // Conexion con dispositivos
  ROS_INFO("[Control] Position / Orientation - Conectando con INS/GPS");
  nodePosOri->setGpsStatus(false);
  // Inicio de artefactos ROS
  nodePosOri->initROS();
  while(!nodePosOri->getGpsStatus() && nodePosOri->getNodeStatus() != NODESTATUS_OFF){
    // Conexion con GPS/INS
    if (nodePosOri->getXSensManager()->doConnect(DEVICE_XSENS)) {
      nodePosOri->setGpsStatus(true);
    } else {
      ROS_INFO("[Control] Position / Orientation - No se puede conectar al GPS/INS");
      usleep(3000000);
      ros::spinOnce();
      ROS_INFO("[Control] Position / Orientation - Imposible conexion con INS/GPS. Reintentando...");
    }
  }

  if (nodePosOri->getNodeStatus() != NODESTATUS_OFF) {
    ROS_INFO("[Control] Position / Orientation - Esperando activacion de nodo");
    while (nodePosOri->getNodeStatus() == NODESTATUS_INIT) {
      ros::spinOnce();
    }
    if (nodePosOri->getNodeStatus() == NODESTATUS_OK) {
      nodePosOri->configureDevices(); 
      ROS_INFO("[Control] Position / Orientation - Nodo activado y listo para operar");
    }
    // Bucle principal Funcionamiento completo (GPS/INS)
    while (ros::ok() && nodePosOri->getNodeStatus() != NODESTATUS_OFF) {
      ros::spinOnce();
      nodePosOri->publishInformation();
      usleep(50000);
    }
    nodePosOri->getXSensManager()->disconnect();
    ROS_INFO("[Control] Position / Orientation - Desconectado dispositivo GPS/INS");
  }
  delete(nodePosOri);
  ROS_INFO("[Control] Position / Orientation - Nodo finalizado");
  return 0;
}