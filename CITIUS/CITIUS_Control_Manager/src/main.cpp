/* 
 * File:   main.cpp
 * Author: atica
 *
 * Created on 17 de julio de 2014, 12:05
 */

#include <cstdlib>
#include "Manager.h"

using namespace std;

/*
 * 
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

