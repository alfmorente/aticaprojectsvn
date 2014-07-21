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
#include "CITIUS_Control_Manager/msg_switcher.h"
#include "constant.h"

class Manager {
public:
    Manager();
    virtual ~Manager();
    void initROS();
    
private:
    // Estado de los demas nodos
    bool positionOrientationOK;
    bool frontCameraOK;
    bool rearCameraOK;
    bool drivingOK;
    
    // Posicion del conmutador Local/teleoperado
    short currentSwitcher;
    
    // Artefactos ROS
    ros::ServiceClient poNodeStatus;
    ros::ServiceClient fcNodeStatus;
    ros::ServiceClient rcNodeStatus;
    ros::ServiceClient drNodeStatus;
    ros::ServiceClient switcher;
    ros::ServiceServer serverVehicleStatus;
    ros::Subscriber switcherLocalTelecontrol;
    
    // Callbacks
    bool fcv_serv_vehicleStatus(CITIUS_Control_Manager::srv_vehicleStatus::Request &rq, CITIUS_Control_Manager::srv_vehicleStatus::Response &rsp);
    void fnc_subs_switcher(CITIUS_Control_Manager::msg_switcher);
};

#endif	/* MANAGER_H */

