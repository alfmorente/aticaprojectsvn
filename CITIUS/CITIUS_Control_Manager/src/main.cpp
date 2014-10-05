
/** 
 * @file  main.cpp
 * @brief Funcion principal del nodo Manager del subsistema de control
 * @author: Carlos Amores
 * @date: 2013, 2014
 */

#include "Manager.h"

/**
 * Metodo principal del nodo. Inicializa la maquina de estados y la pone en 
 * marcha gestionando el estado del resto de nodos del vehiculo
 * @param[in] argc Numero de argumentos
 * @param[in] argv Vector de argumentos
 * @return Entero distinto de 0 si ha habido problemas. 0 en caso contrario.
 */
int main(int argc, char** argv) {
    
    ros::init(argc, argv, "Control_ROS_Node_Manager");
    
    // Creacion de la maquina de estados
    Manager *manager = new Manager();
    
    // Iniciar artefactos ROS
    manager->initROS();
    ROS_INFO("[Control] Manager - Nodo listo para operar");
    
    int status = -1;
    ros::NodeHandle nh;
    nh.setParam("vehicleStatus",status);
    ROS_INFO("[Control] Manager - Esperando conexion con nodo Electric");
    while(status == -1){
        nh.getParam("vehicleStatus",status);
        ros::spinOnce();
    }
    ROS_INFO("[Control] Manager - Maquina de estados iniciada");
    
    // Bucle principal
    while(ros::ok() && status!=OPERATION_MODE_APAGANDO){
        nh.getParam("vehicleStatus",status);
        ros::spinOnce();
        
    }
    ROS_INFO("[Control] Manager - Nodo finalizado");
        
    return 0;
}

