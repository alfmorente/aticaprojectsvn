
/** 
 * @file  main.cpp
 * @brief Función principal del nodo RearCamera del subsistema de control
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "RosNode_RearCamera.h"

/**
 * Metodo principal del nodo. Inicializa modulos ROS y lanza el intercambio de 
 * mensajes con la cámara
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "RosNode_Control_RearCam");
  RosNode_RearCamera *rc = new RosNode_RearCamera();
  // Espera conexion cada segundo
  ROS_INFO("[Control] RearCamera - Esperando conexion con camara");
  while (!rc->getDriverMng()->checkConnection()) {
    usleep(3000000);
    ROS_INFO("[Control] RearCamera - Imposible conexion. Reintentando...");
  }
  rc->initROS();
  ROS_INFO("[Control] RearCamera - Esperando activacion de nodo");
  while (rc->getNodeStatus() != NODESTATUS_OK) {
    ros::spinOnce();
  }
  ROS_INFO("[Control] RearCamera - Nodo listo para operar");
  // Temporizador de requerimiento de informacion
  Timer *timer = new Timer();
  timer->Enable();
  while (ros::ok() && rc->getNodeStatus() != NODESTATUS_OFF) {
    ros::spinOnce();
    if (timer->GetTimed() >= FREC_2HZ) {
      timer->Reset();
      rc->manageDevice();
    }
  }
  delete(timer);
  delete(rc);
  ROS_INFO("[Control] RearCamera - Nodo finalizado");
  return 0;
}

