
/** 
 * @file  main.cpp
 * @brief Función principal del nodo de Comunicaciones del subsistema de control
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode_Communications.h"

/**
 * Método principal del nodo. Inicializa modulos ROS y JAUS y lanza el
 * intercambio y la traducción de mensajes
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "Control_ROS_Node_Communications");
  RosNode_Communications *nodeComm = RosNode_Communications::getInstance();
  nodeComm->initROS();
  nodeComm->initJAUS();
  ROS_INFO("[Control] Communications - Esperando activacion de nodo");
  while (nodeComm->getNodeStatus() == NODESTATUS_INIT) {
    ros::spinOnce();
  }
  ROS_INFO("[Control] Communications - Esperando controlador (MyC / Tablet)...");
  while(!nodeComm->isControllerAvailable() && nodeComm->getNodeStatus() != NODESTATUS_OFF){
      ros::spinOnce();
    usleep(100000);
  }
  ROS_INFO("[Control] Communications - Nodo listo para operar");
  ros::NodeHandle nh;
  // Temporizador de envio de estado
  Timer *timer = new Timer();
  timer->Enable();
  int count1Hz = 0;
  while (ros::ok() && nodeComm->getNodeStatus() != NODESTATUS_OFF) {
    // Recepcion de mensajeria ROS
    ros::spinOnce();
    // Temporizador para envio de estado
    if (timer->GetTimed() >= FREC_10HZ) {
      // Clear del timer
      timer->Reset();
      // Informe del estado del modo de operacion
      nodeComm->informStatus();
      // Temporizador para envio de identificador de camara activa a pinchar
      count1Hz++;
      if(count1Hz==10){
        count1Hz = 0;
        // Informe de la camara activa a pinchar
        nodeComm->informCameraToStream();
        // Informe de heartbeat (a Comm Mngmt de todos los subsistemas)
        nodeComm->informHeartbeatPositionInfo();
      }
    }
    usleep(50000);
  }
  nodeComm->finishJAUS();
  delete(timer);
  delete(nodeComm);
  ROS_INFO("[Control] Communications - Nodo finalizado");
  return 0;
}

