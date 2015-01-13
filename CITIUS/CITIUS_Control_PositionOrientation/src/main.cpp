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
  // Conexion con GPS/INS
  if (nodePosOri->getXSensManager()->doConnect(DEVICE_XSENS)) {
    ROS_INFO("[Control] Position / Orientation - Conectado a dispositivo GPS/INS");
    nodePosOri->setGpsStatus(true);
  } else {
    ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo GPS/INS");
    nodePosOri->setGpsStatus(false);
  }
  // Conexion con magnetometro
  if (nodePosOri->getMagnetometerManager()->doConnect(DEVICE_AHRS)) {
    ROS_INFO("[Control] Position / Orientation - Conectado a dispositivo Magnetometro");
    nodePosOri->setMagnStatus(true);
  } else {
    ROS_INFO("[Control] Position / Orientation - No se puede conectar al dispositivo Magnetometro");
    nodePosOri->setMagnStatus(false);
  }
  // Check de conexion de dispositivos
  if (nodePosOri->getGpsStatus() || nodePosOri->getMagnStatus()) {
    // Inicio de artefactos ROS
    nodePosOri->initROS();
    ROS_INFO("[Control] Position / Orientation - Esperando activacion de nodo");
    while (nodePosOri->getNodeStatus() == NODESTATUS_INIT) {
      ros::spinOnce();
      //printf("%d\n",nodePosOri->getNodeStatus());
    }
    if (nodePosOri->getNodeStatus() == NODESTATUS_OK) {
      nodePosOri->configureDevices();
      ROS_INFO("[Control] Position / Orientation - Nodo activado y listo para operar");

    }
    // Bucle principal Funcionamiento completo (GPS/INS + Mag)
    if (nodePosOri->getGpsStatus() && nodePosOri->getMagnStatus()) {
      ROS_INFO("[Control] Position / Orientation - Funcionando con GPS/INS y Magnetometro");
      while (ros::ok() && nodePosOri->getNodeStatus() != NODESTATUS_OFF) {
        ros::spinOnce();
        nodePosOri->publishInformation();
        usleep(50000);
      }
    }      
    // Bucle principal Funcionamiento parcial solo GPS/INS
    else if (nodePosOri->getGpsStatus() && !nodePosOri->getMagnStatus()) {
      ROS_INFO("[Control] Position / Orientation - Funcionando solo con GPS/INS");
      while (ros::ok() && nodePosOri->getNodeStatus() != NODESTATUS_OFF) {
        ros::spinOnce();
        nodePosOri->publishInformation();
        usleep(50000);
      }
    }      
    // Bucle principal Funcionamiento parcial solo Mag
    else if (!nodePosOri->getGpsStatus() && nodePosOri->getMagnStatus()) {
      Timer *timer = new Timer();
      timer->Enable();
      ROS_INFO("[Control] Position / Orientation - Funcionando solo con Magnetometro");
      while (ros::ok() && nodePosOri->getNodeStatus() != NODESTATUS_OFF) {
        ros::spinOnce();
        if (timer->GetTimed() >= FREC_10HZ) {
          nodePosOri->publishInformation();
          usleep(50000);
        }
      }
      delete(timer);
    }
    nodePosOri->getXSensManager()->disconnect();
    ROS_INFO("[Control] Position / Orientation - Desconectado dispositivo GPS/INS");
  }
  delete(nodePosOri);
  ROS_INFO("[Control] Position / Orientation - Nodo finalizado");
  return 0;
}