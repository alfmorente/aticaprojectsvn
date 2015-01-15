
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

  ros::init(argc, argv, "Control_ROS_Node_Electric");
  RosNode_Electric *nodeElectric = new RosNode_Electric();

  // Espera conexion cada segundo
  ROS_INFO("[Control] Electric - Esperando conexion con vehiculo");
  while (!nodeElectric->getDriverMng()->doConnect(DEVICE_ELECTRIC)){
    usleep(1000000);
    ROS_INFO("[Control] Electric - Imposible conexion. Reintentando...");
  }

  ROS_INFO("[Control] Electric - Conexion establecida con vehiculo");
  nodeElectric->initROS();
  ROS_INFO("[Control] Electric - Esperando la activacion del nodo");
  while (nodeElectric->getNodeStatus() == NODESTATUS_INIT) {
    ros::spinOnce();
  }
  if (nodeElectric->getNodeStatus() == NODESTATUS_OK) {
    ROS_INFO("[Control] Electric - Nodo listo para operar");
  }
  // Temporizador de requerimiento de informacion
  Timer *timer = new Timer();
  timer->Enable();

  while (ros::ok() && nodeElectric->getNodeStatus() != NODESTATUS_OFF) {
    ros::spinOnce();
    // Funcionamiento correcto (sin alarmas)
    if (nodeElectric->getNodeStatus() == NODESTATUS_OK) {
      nodeElectric->getDriverMng()->checkForVehicleMessages();
      nodeElectric->checkTurnOff();

      nodeElectric->checkSupplyAlarms();
      if (timer->GetTimed() >= FREC_2HZ) {
        timer->Reset();
        nodeElectric->getDriverMng()->reqElectricInfo();
        nodeElectric->publishElectricInfo(nodeElectric->getDriverMng()->getVehicleInfo());
      }
    }      // Funcionamiento en modo degradado 
    else if (nodeElectric->getNodeStatus() == NODESTATUS_CORRUPT) {
      nodeElectric->checkSupplyAlarms();
      nodeElectric->getDriverMng()->checkForVehicleMessages();
      nodeElectric->checkTurnOff();
    }
    usleep(1000);
  }
  
  delete(timer);
  delete(nodeElectric);
  ROS_INFO("[Control] Electric - Nodo finalizado");
  return 0;
}