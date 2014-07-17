/* 
 * File:   Manager.h
 * Author: atica
 *
 * Created on 17 de julio de 2014, 11:47
 */

#ifndef MANAGER_H
#define	MANAGER_H

#include "ros/ros.h"
#include "CITIUS_Control_Manager/srv_nodeStatus.h"
#include "CITIUS_Control_Manager/srv_vehicleStatus.h"

class Manager {
public:
    Manager();
    Manager(const Manager& orig);
    virtual ~Manager();
    void initROS();
    
private:
    ros::ServiceClient poNodeStatus;
    ros::ServiceClient fcNodeStatus;
    ros::ServiceClient rcNodeStatus;
    ros::ServiceClient drNodeStatus;
    ros::ServiceServer serverNodeStatus;
    ros::ServiceServer serverVehicleStatus;
    
    bool fcv_serv_nodeStatus(CITIUS_Control_Manager::srv_nodeStatus::Request &rq, CITIUS_Control_Manager::srv_nodeStatus::Response &rsp);
    bool fcv_serv_vehicleStatus(CITIUS_Control_Manager::srv_vehicleStatus::Request &rq, CITIUS_Control_Manager::srv_vehicleStatus::Response &rsp);

};

#endif	/* MANAGER_H */

