
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

  // Iniciacion del middleware (ROS) para el nodo Communications
  ros::init(argc, argv, "Control_ROS_Node_Communications");
  RosNode_Communications *nodeComm = RosNode_Communications::getInstance();
  nodeComm->initROS();
  nodeComm->initJAUS();

  // Espera a permiso para comenzar a operar ROSNODE_OK
  ROS_INFO("[Control] Communications - Esperando activacion de nodo");
  while (nodeComm->getNodeStatus() == NODESTATUS_INIT) {
    ros::spinOnce();
  }

  ROS_INFO("[Control] Communications - Nodo listo para operar");

  // Control del modo de operacion del vehiculo
  ros::NodeHandle nh;

  // Temporizador de envio de estado
  Timer *timer = new Timer();
  timer->Enable();

  while (ros::ok() && nodeComm->getNodeStatus() != NODESTATUS_OFF) {

    // Recepcion de mensajeria ROS
    ros::spinOnce();

    // Temporizador para envio de estado
    if (timer->GetTimed() >= FREC_10HZ) {

      // Clear del timer
      timer->Reset();
      // Requerimiento de informacion de dispositivo
      nodeComm->informStatus();

    }
    usleep(50000);

  }
  nodeComm->finishJAUS();
  delete(timer);
  delete(nodeComm);

  ROS_INFO("[Control] Communications - Nodo finalizado");

  return 0;
}

