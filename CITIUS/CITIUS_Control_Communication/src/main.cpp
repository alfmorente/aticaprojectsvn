
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

  ROS_INFO("[Control] Communications - Nodo listo para operar");

  // Temporizador de envio de estado
  clock_t initTime, finalTime;
  initTime = clock();

  // Control del modo de operacion del vehiculo
  int currentStatus = OPERATION_MODE_INICIANDO;
  ros::NodeHandle nh;

  while (ros::ok() && currentStatus != OPERATION_MODE_APAGANDO) {

    // Recepcion de mensajeria ROS
    ros::spinOnce();

    // Temporizador para envio de estado
    finalTime = clock() - initTime;
    if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_10HZ) {

      // Clear del timer
      initTime = clock();
      // Requerimiento de informacion de dispositivo
      nodeComm->informStatus();

    }

    nh.getParam("vehicleStatus", currentStatus);

  }
  nodeComm->finishJAUS();
  delete(nodeComm);

  ROS_INFO("[Control] Communications - Nodo finalizado");

  return 0;
}

