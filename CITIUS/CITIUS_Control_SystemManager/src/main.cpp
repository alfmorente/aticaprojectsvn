/* 
 * File:   main.cpp
 * Author: Carlos Amores
 *
 * Created on 5 de junio de 2014, 9:06
 */

#include "ros/ros.h"
#include <cstdlib>
#include "CITIUS_Control_SystemManager/Manager.h"

using namespace std;

int main(int argc, char** argv) {

    // Iniciacion del middleware (ROS) para el nodo Driving
    ros::init(argc,argv,"ROSNODE_Control_Manager");
    
    // Con la creacion de msg, se crean e inician servidores y clientes
    Manager *mng = new Manager();
    
    // Habilitada la recepcion de mensajes/servicios
    ros::spin();
    
    ROS_INFO("[Control] Manager :: Nodo finalizado");
    
    return 0;
}