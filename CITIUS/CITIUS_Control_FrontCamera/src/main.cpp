
/** 
 * @file  main.cpp
 * @brief Función principal del nodo FrontCamera del subsistema de control
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode_FrontCamera.h"

/**
 * Metodo principal del nodo. Inicializa modulos ROS y lanza el intercambio de 
 * mensajes con la cámara
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {

  ros::init(argc, argv, "RosNode_Control_FrontCam");

  RosNode_FrontCamera *fc = new RosNode_FrontCamera();

  // Conexión
  if (fc->getDriverMng()->checkConnection()) {
    // Inicio de artefactos ROS
    fc->initROS();

    // Espera a permiso para comenzar a operar ROSNODE_OK
    ROS_INFO("[Control] FrontCamera - Esperando activacion de nodo");
    while (fc->getNodeStatus() != NODESTATUS_OK) {
      ros::spinOnce();
    }
    ROS_INFO("[Control] FrontCamera - Nodo listo para operar");

    // Temporizador de requerimiento de informacion
    Timer *timer = new Timer();
    timer->Enable();

    //Bucle principal
    while (ros::ok() && fc->getNodeStatus() != NODESTATUS_OFF) {

      // Recepcion de mensaje
      ros::spinOnce();
      
      // Comprobación del temporizador y requerimiento de info
      if (timer->GetTimed() >= FREC_2HZ) {

        // Clear del timer
          timer->Reset();
        // Envio de estado, requerimiento de info y publicacion
        fc->manageDevice();

      }
    }
    delete(timer);
  } else {
    ROS_INFO("[Control] FrontCamera - No se puede conectar con la camara");
  }
  delete(fc);
  ROS_INFO("[Control] FrontCamera - Nodo finalizado");
  return 0;
}

