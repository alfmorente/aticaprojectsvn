
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

  // Espera conexion cada segundo
  ROS_INFO("[Control] FrontCamera - Esperando conexion con camara");
  while (!fc->getDriverMng()->checkConnection()) {
    usleep(3000000);
    ROS_INFO("[Control] FrontCamera - Imposible conexion. Reintentando...");
  }
  fc->initROS();
  ROS_INFO("[Control] FrontCamera - Esperando activacion de nodo");
  while (fc->getNodeStatus() != NODESTATUS_OK) {
    ros::spinOnce();
  }
  ROS_INFO("[Control] FrontCamera - Nodo listo para operar");
  Timer *timer = new Timer();
  timer->Enable();
  while (ros::ok() && fc->getNodeStatus() != NODESTATUS_OFF) {
    ros::spinOnce();
    if (timer->GetTimed() >= FREC_2HZ) {
      timer->Reset();
      fc->manageDevice();
    }
  }
  delete(timer);
  delete(fc);
  ROS_INFO("[Control] FrontCamera - Nodo finalizado");
  return 0;
}

