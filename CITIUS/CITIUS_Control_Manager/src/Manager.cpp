/* 
 * File:   Manager.cpp
 * Author: atica
 * 
 * Created on 17 de julio de 2014, 11:47
 */

#include "Manager.h"

Manager::Manager() {
    
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
    serverNodeStatus = nh.advertiseService("toNodeStatus",&Manager::fcv_serv_nodeStatus,this);
    serverVehicleStatus = nh.advertiseService("vehicleStatus",&Manager::fcv_serv_vehicleStatus,this);
}

bool Manager::fcv_serv_nodeStatus(CITIUS_Control_Manager::srv_nodeStatus::Request &rq, CITIUS_Control_Manager::srv_nodeStatus::Response &rsp){
    return true;
}

bool Manager::fcv_serv_vehicleStatus(CITIUS_Control_Manager::srv_vehicleStatus::Request &rq, CITIUS_Control_Manager::srv_vehicleStatus::Response &rsp){
    return true;
}

