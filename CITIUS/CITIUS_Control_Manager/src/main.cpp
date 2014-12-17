
/** 
 * @file  main.cpp
 * @brief Función principal del nodo Manager del subsistema de control
 * @author Carlos Amores
 * @date 2013, 2014
 */

#include "Manager.h"

/**
 * Método principal del nodo. Inicializa la máquina de estados y la pone en 
 * marcha gestionando el estado del resto de nodos del vehículo
 * @param[in] argc Número de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "Control_ROS_Node_Manager");
  Manager *manager = new Manager();
  manager->initROS();
  ROS_INFO("[Control] Manager - Nodo listo para operar");
  int status = -1;
  ros::NodeHandle nh;
  nh.setParam("vehicleStatus", status);
  ROS_INFO("[Control] Manager - Esperando conexion con vehiculo");
  while (status == -1) {
    nh.getParam("vehicleStatus", status);
    ros::spinOnce();
  }
  manager->checkPreviousExec();
  ROS_INFO("[Control] Manager - Maquina de estados iniciada");
  while (ros::ok() && status != OPERATION_MODE_APAGANDO) {
    nh.getParam("vehicleStatus", status);
    ros::spinOnce();
    usleep(100000);
  }
  delete(manager);
  ROS_INFO("[Control] Manager - Nodo finalizado");
  return 0;
}

