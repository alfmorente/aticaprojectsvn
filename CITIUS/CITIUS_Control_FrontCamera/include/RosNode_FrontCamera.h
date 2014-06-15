/* 
 * File:   RosNode_FrontCamera.h
 * Author: Carlos Amores
 *
 * Created on 15 de junio de 2014, 16:17
 */

#ifndef ROSNODE_FRONTCAMERA_H
#define	ROSNODE_FRONTCAMERA_H

#ifdef	__cplusplus
extern "C" {
#endif




#ifdef	__cplusplus
}
#endif

#endif	/* ROSNODE_FRONTCAMERA_H */

#include <time.h>
#include "DrivingCameraManager.h"
#include "constant.h"
#include "CITIUS_Control_FrontCamera/msg_ctrlFrontCamera.h"
#include "CITIUS_Control_FrontCamera/msg_frontCameraInfo.h"
#include "CITIUS_Control_FrontCamera/srv_nodeStatus.h"

class RosNode_FrontCamera{
private:
    // Estado del nodo
    short fcNodeStatus;
    // Publicador de informacion de camara
    ros::Publisher pubFrontCameraInfo;
    // Suscriptor para control de camara
    ros::Subscriber subsCtrlFrontCamera;
    // Servidor de estado de nodo
    ros::ServiceServer servNodeStatus;
    // Driver de la cámara
    DrivingCameraManager *dFrontCamera;
public:
    // Constructor
    RosNode_FrontCamera();
    // Inicializador de artefactos ROS
    void initROS();
    // Callbacks ROS
    void fcn_sub_ctrlFrontCamera(CITIUS_Control_FrontCamera::msg_ctrlFrontCamera msg);
    bool fcv_serv_nodeStatus(CITIUS_Control_FrontCamera::srv_nodeStatus::Request &rq, CITIUS_Control_FrontCamera::srv_nodeStatus::Response &rsp);
    // Getter and Setter necesarios
    ros::Publisher getPubFrontCameraInfo();
    short getFcNodeStatus();
    void setFcNodeStatus(short newFcNodeStatus);
    DrivingCameraManager *getDriverMng();
    
};

