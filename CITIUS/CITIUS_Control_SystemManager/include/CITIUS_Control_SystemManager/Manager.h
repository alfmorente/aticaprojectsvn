/* 
 * File:   Manager.h
 * Author: Carlos Amores (AICIA)
 *
 * Created on 13 de mayo de 2014, 12:39
 */

#ifndef MANAGER_H
#define	MANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* MANAGER_H */

#include "ros/ros.h"
#include "CITIUS_Control_SystemManager/srv_nodeStatus.h"
#include "CITIUS_Control_SystemManager/srv_vehicleStatus.h"
#include "constant.h"

/*******************************************************************************
 *              CLASE MANEJADOR DE CONEXION CON DRIVING
*******************************************************************************/

class Manager{
    public:
        // Constructor
        Manager();        
        // Callbacks
        bool fcn_serv_nodeStatus(CITIUS_Control_SystemManager::srv_nodeStatus::Request &rq, CITIUS_Control_SystemManager::srv_nodeStatus::Response &rsp);
        bool fcn_serv_vehicleStatus(CITIUS_Control_SystemManager::srv_vehicleStatus::Request &rq, CITIUS_Control_SystemManager::srv_vehicleStatus::Response &rsp);
    private:
        // Manejadores de nodos ROS
        ros::NodeHandle nh;
        // Servidores
        ros::ServiceServer server_nodeStatus;
        ros::ServiceServer server_vehicleStatus;
        // Clientes
        ros::ServiceClient client_fCamera_nodeStatus;
        ros::ServiceClient client_rCamera_nodeStatus;
        ros::ServiceClient client_posOri_nodeStatus;
        ros::ServiceClient client_driving_nodeStatus;
};