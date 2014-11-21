
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
  short numOfAttemps = 0;
  ROS_INFO("[Control] Electric - Conectando con subsistema de gestion de energia");
  while (!nodeElectric->getDriverMng()->doConnect(DEVICE_ELECTRIC) && numOfAttemps < MAX_ATTEMPS) {
    numOfAttemps++;
  }
  if (numOfAttemps < MAX_ATTEMPS) {
    ROS_INFO("[Control] Electric - Conexion establecida con vehiculo");
    nodeElectric->initROS();
    if (nodeElectric->getNodeStatus() != NODESTATUS_OFF) {
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
        nodeElectric->checkSwitcher();
        nodeElectric->checkSupplyAlarms();
        if (timer->GetTimed() >= FREC_2HZ) {
          timer->Reset();
          nodeElectric->getDriverMng()->reqElectricInfo();
          nodeElectric->publishElectricInfo(nodeElectric->getDriverMng()->getVehicleInfo());
        }
      }
      // Funcionamiento en modo degradado 
      else if(nodeElectric->getNodeStatus() == NODESTATUS_CORRUPT){
        nodeElectric->checkSupplyAlarms();
        nodeElectric->getDriverMng()->checkForVehicleMessages();
        nodeElectric->checkTurnOff();
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