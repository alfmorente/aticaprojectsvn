/* 
 * File:   Manager.cpp
 * Author: atica
 * 
 * Created on 17 de julio de 2014, 11:47
 */

#include "Manager.h"

Manager::Manager() {
    ros::NodeHandle nh;
    nh.se
}

Manager::Manager(const Manager& orig) {
}

Manager::~Manager() {
}

void Manager::initROS(){
    ros::NodeHandle nh;
    poNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("poNodeStatus");
    fcNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("fcNodeStatus");
    rcNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("rcNodeStatus");
    drNodeStatus = nh.serviceClient<CITIUS_Control_Manager::srv_nodeStatus>("drNodeStatus");
    serverVehicleStatus = nh.advertiseService("vehicleStatus",&Manager::fcv_serv_vehicleStatus,this);
}

bool Manager::fcv_serv_vehicleStatus(CITIUS_Control_Manager::srv_vehicleStatus::Request &rq, CITIUS_Control_Manager::srv_vehicleStatus::Response &rsp){
    
    // Variable de devolucion
    bool dev = true;
    
    // Manejador ROS
    ros::NodeHandle nh;
    
    // Lectura del estado actual
    int currentStatus;
    nh.getParam("vehicleStatus",currentStatus);
    
    // Maquina de estados
    switch(rq.status){
        case OPERATION_MODE_INICIANDO:
            dev = false;
            rsp.confirmation = false;
            break;
        case OPERATION_MODE_LOCAL:
            break;
        case OPERATION_MODE_CONDUCCION:
            break;
        case OPERATION_MODE_OBSERVACION:
            break;
        case OPERATION_MODE_APAGANDO:
            break;
        default:
            break;
    }
    
    return dev;
}

