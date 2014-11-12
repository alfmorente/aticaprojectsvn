
/** 
 * @file  main.cpp
 * @brief Funcion principal del nodo Driving del subsistema de control
 * @author Carlos Amores
 * @date 2013, 2014
 */


#include "RosNode_Electric.h"

/**
 * Metodo principal del nodo. Inicializa modulos ROS y lanza el
 * intercambio y la recepcion de mensajes con el vehiculo y su logica de
 * negocio
 * @param[in] argc Numero de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {

  // Iniciacion del middleware (ROS) para el nodo Driving
  ros::init(argc, argv, "Control_ROS_Node_Electric");

  RosNode_Electric *nodeElectric = new RosNode_Electric();

  // Intentos de reconexion
  short numOfAttemps = 0;
  ROS_INFO("[Control] Electric - Conectando con subsistema de gestion de energia");
  while (!nodeElectric->getDriverMng()->doConnect(DEVICE_ELECTRIC) && numOfAttemps < MAX_ATTEMPS) {
    numOfAttemps++;
  }

  if (numOfAttemps < MAX_ATTEMPS) {
    
    ROS_INFO("[Control] Electric - Conexion establecida con vehiculo");
    
    // Inicio de artefactos ROS
    nodeElectric->initROS();

    // Espera a permiso para comenzar a operar ROSNODE_OK
    if (nodeElectric->getNodeStatus() != NODESTATUS_OFF) {
      ROS_INFO("[Control] Electric - Nodo listo para operar");
    }

    // Temporizador de requerimiento de informacion
    Timer *timer = new Timer();
    timer->Enable();

    //Bucle principal
    while (ros::ok() && nodeElectric->getNodeStatus() != NODESTATUS_OFF) {

      // Recepcion de mensaje ROS
      ros::spinOnce();

      // Funcionamiento correcto (sin alarmas)
      if (nodeElectric->getNodeStatus() == NODESTATUS_OK) {

        // Comprobacion de recpcion de mensajes de vehiculo
        nodeElectric->getDriverMng()->checkForVehicleMessages();
        // Comprobacion de señalizacion de apagado
        nodeElectric->checkTurnOff();
        // Comprobacion de cambio en posicion conmutador local/teleop
        nodeElectric->checkSwitcher();
        // Comprobacion de cambio en los vectores de alarmas
        nodeElectric->checkSupplyAlarms();

        // Comprobación del temporizador y requerimiento de info
        if (timer->GetTimed() >= FREC_2HZ) {
          // Clear del timer
          timer->Reset();
          // Requerimiento de informacion de sistema energetico
          nodeElectric->getDriverMng()->reqElectricInfo();
          // Publicacion de la informacion existente
          nodeElectric->publishElectricInfo(nodeElectric->getDriverMng()->getVehicleInfo());
        }
      
      }
      // Funcionamiento en modo degradado 
      else if(nodeElectric->getNodeStatus() == NODESTATUS_CORRUPT){
        // Comprobacion de cambio en los vectores de alarmas
        nodeElectric->checkSupplyAlarms();
        // Comprobacion recepción de nuevo vector de alarmas o apagado
        nodeElectric->getDriverMng()->checkForVehicleMessages();
        // Comprobqacion de señalizacion de apagado
        nodeElectric->checkTurnOff();
        // Comprobacion de cambio en posicion conmutador local/teleop
        nodeElectric->checkSwitcher();
      }
      
      usleep(1000);
    }
    delete(timer);
  } else {
    ROS_INFO("[Control] Electric - No se puede conectar al vehiculo. Maximo numero de reintentos realizados.");
  }

  delete(nodeElectric);
  ROS_INFO("[Control] Electric - Nodo finalizado");
  return 0;
}