/* 
 * File:   ROSNode_Manager.h
 * Author: Carlos Amores
 *
 * Created on 14 de mayo de 2014, 20:55
 */

#ifndef ROSNODE_MANAGER_H
#define	ROSNODE_MANAGER_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* ROSNODE_MANAGER_H */

#include "ros/ros.h"
#include "CITIUS_Control_Manager/srv_status.h"
#include "CITIUS_Control_Manager/srv_electric.h"
#include "CITIUS_Control_Manager/srv_frontcam.h"
#include "CITIUS_Control_Manager/srv_rearcam.h"
#include "CITIUS_Control_Manager/srv_vehicle.h"
#include "CITIUS_Control_Manager/msg_cameraStatus.h"
#include "CITIUS_Control_Manager/msg_vehicleInfo.h"
#include "CITIUS_Control_Manager/msg_command.h"
#include "CITIUS_Control_Manager/msg_electricInfo.h"

class ROSNode_Manager{
        private:
            ros::NodeHandle nh;
            ros::ServiceServer serverStatus;
            ros::ServiceClient clientVehicle;
            ros::ServiceClient clientElectric;
            ros::ServiceClient clientFrontCamera;
            ros::ServiceClient clientRearCamera;
            ros::Publisher publisherFrontCamStatus;
            ros::Publisher publisherRearCamStatus;
            ros::Publisher publisherVehicleInfo;
            ros::Publisher publisherElectricInfo;
            ros::Publisher publisherCommand;
            ros::Subscriber subscriberCommand;
        public:
            ROSNode_Manager();
            void initialize();
            void run();
};

