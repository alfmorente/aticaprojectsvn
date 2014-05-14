/* 
 * File:   main.cpp
 * Author: Carlos Amores (AICIA)
 *
 * Created on 13 de mayo de 2014, 12:34
 */

#include <cstdlib>
#include "CITIUS_Control_Manager/Manager.h"

using namespace std;

// Servicio para el cambio de estado / subestado

bool callback_status(CITIUS_Control_Manager::srv_status::Request &rq, CITIUS_Control_Manager::srv_status::Response &rs){
    ros::NodeHandle localNh;
    int estado_actual;
    ROS_INFO("===================================================");
    localNh.getParam("status",estado_actual);
    ROS_INFO("El estado actual es %d\n", estado_actual);
    localNh.setParam("status",rq.status);
    localNh.getParam("status",estado_actual);
    ROS_INFO("Cambiado a: %d\n", estado_actual);
    ROS_INFO("===================================================");
    rs.confirmation = true;
    return true;
}

int main(int argc, char** argv) {
    
    ros::init(argc,argv,"CITIUS_Manager");
    ros::NodeHandle nh;
    
    nh.setParam("status",0);
    
    /* Definicion e inicializacion de servicios 
     * - service_status
     */

    ros::ServiceServer srvStatus = nh.advertiseService("service_status", callback_status);
    
    /* 
     *
     */
    ROS_INFO("Preparado para dar servicio al cambio de estado...");
    ros::spin();
    return 0;
}