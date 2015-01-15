
/** 
 * @file  main.cpp
 * @brief Funcion principal del nodo Driving del subsistema de control
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode_Driving.h"

/**
 * Metodo principal del nodo. Inicializa modulos ROS y lanza el
 * intercambio de mensajes con el vehículo
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {

  // Iniciacion del middleware (ROS) para el nodo Driving
  ros::init(argc, argv, "RosNode_Control_Driving");
  RosNode_Driving *nodeDriving = new RosNode_Driving();

  // Espera conexion cada segundo
  ROS_INFO("[Control] Driving - Esperando conexion con vehiculo");
  while (!nodeDriving->getDriverMng()->doConnect(DEVICE_DRIVING)){
    usleep(1000000);
    ROS_INFO("[Control] Driving - Imposible conexion. Reintentando...");
  }

  ROS_INFO("[Control] Driving - Establecida conexion con vehiculo");
  nodeDriving->initROS();

  // Conteo de 5 hz -> 5 * 1hz (Vehiculo 5Hz, Senalizacion 1Hz))
  short hzCount = 0;
  // Temporizador de requerimiento de informacion
  Timer *timer = new Timer();
  timer->Enable();
  //Bucle principal
  while (ros::ok() && nodeDriving->getNodeStatus() != NODESTATUS_OFF) {
    if (nodeDriving->getDriverMng()->getSocketDescriptor() == -1) {
      ROS_INFO("[Control] Driving - Se ha perdido comunicacion con servidor");
    } else {
      // Comprobacion de recepcion de mensaje ROS
      ros::spinOnce();
      // Funcionamiento normal (sin alarms)
      if (nodeDriving->getNodeStatus() == NODESTATUS_OK) {
        nodeDriving->getDriverMng()->checkForVehicleMessages();
        nodeDriving->checkAlarms();
        nodeDriving->checkSwitcher();
        // Comprobación del temporizador y requerimiento de info
        if (timer->GetTimed() >= FREC_5HZ) {
          // Clear del timer
          timer->Reset();
          hzCount++;
          if (hzCount == 5) {
            hzCount = 0;
            nodeDriving->getDriverMng()->reqVehicleInfo(true);
            nodeDriving->publishDrivingInfo(nodeDriving->getDriverMng()->getVehicleInfo(true));
          } else {
            nodeDriving->getDriverMng()->reqVehicleInfo(false);
            nodeDriving->publishDrivingInfo(nodeDriving->getDriverMng()->getVehicleInfo(false));
          }
        }
      }        // Funcionamiento en modo degradado (alarmas)
      else if (nodeDriving->getNodeStatus() == NODESTATUS_CORRUPT) {
        nodeDriving->getDriverMng()->checkForVehicleMessages();
        nodeDriving->checkAlarms();
        nodeDriving->checkSwitcher();
      }
    }
    usleep(1000);
  }
  delete(timer);
  usleep(3000000);
  delete(nodeDriving);
  ROS_INFO("[Control] Driving - Nodo finalizado");
  return 0;
}
