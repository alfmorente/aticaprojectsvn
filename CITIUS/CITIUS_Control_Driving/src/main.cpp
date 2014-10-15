
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

  if (nodeDriving->getDriverMng()->connectVehicle()) {
    // Inicio de artefactos ROS
    nodeDriving->initROS();

    // Espera a permiso para comenzar a operar ROSNODE_OK
    ROS_INFO("[Control] Driving - Esperando activacion de nodo");
    while (nodeDriving->getNodeStatus() == NODESTATUS_INIT) {
      ros::spinOnce();
    }
    if (nodeDriving->getNodeStatus() == NODESTATUS_OK)
      ROS_INFO("[Control] Driving - Nodo listo para operar");

    // Temporizador de requerimiento de informacion
    clock_t initTime, finalTime;
    initTime = clock();

    // Conteo de 5 hz -> 5 * 1hz (Vehiculo 5Hz, Senalizacion 1Hz))
    short hzCount = 0;

    //Bucle principal
    while (ros::ok() && nodeDriving->getNodeStatus() != NODESTATUS_OFF) {
      if (nodeDriving->getDriverMng()->getSocketDescriptor() == -1) {
        ROS_INFO("[Control] Driving - Se ha perdido comunicacion con servidor");
      } else {

        // Comprobacion de recepcion de mensaje ROS
        ros::spinOnce();

        // Comprobacion de recpcion de mensajes de vehiculo
        nodeDriving->getDriverMng()->checkForVehicleMessages();

        // Comprobacion de alarmas
        // TODO

        // Comprobación del temporizador y requerimiento de info
        finalTime = clock() - initTime;
        if (((double) finalTime / ((double) CLOCKS_PER_SEC)) >= FREC_5HZ) {

          // Clear del timer
          initTime = clock();

          hzCount++;
          DrivingInfo info;
          if (hzCount == 5) {

            hzCount = 0;
            // Informacion completa: dispositivos y senalizacion
            // Requiere al vehiculo (GET)
            nodeDriving->getDriverMng()->reqVehicleInfo(true);
            // Obtiene informacion existente para publicar
            info = nodeDriving->getDriverMng()->getVehicleInfo(true);

          } else {

            // Informacion basica: dispositivos
            // Requiere al vehiculo (GET)
            nodeDriving->getDriverMng()->reqVehicleInfo(false);
            // Obtiene informacion existente para publicar
            info = nodeDriving->getDriverMng()->getVehicleInfo(false);

          }
          nodeDriving->publishDrivingInfo(info);

        }
      }

    }
  } else {
    ROS_INFO("[Control] Driving - No se puede conectar al vehiculo");
  }
  ROS_INFO("[Control] Driving - Nodo finalizado");
  return 0;
}
