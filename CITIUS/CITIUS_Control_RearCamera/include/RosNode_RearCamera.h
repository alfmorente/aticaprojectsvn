/* 
 * File:   RosNode_RearCamera.h
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
#include "CITIUS_Control_RearCamera/msg_ctrlRearCamera.h"
#include "CITIUS_Control_RearCamera/msg_rearCameraInfo.h"
#include "CITIUS_Control_RearCamera/srv_nodeStatus.h"

class RosNode_RearCamera{
private:
    // Estado del nodo
    short fcNodeStatus;
    // Publicador de informacion de camara
    ros::Publisher pubRearCameraInfo;
    // Suscriptor para control de camara
    ros::Subscriber subsCtrlRearCamera;
    // Servidor de estado de nodo
    ros::ServiceServer servNodeStatus;
    // Driver de la cámara
    DrivingCameraManager *dRearCamera;
public:
    // Constructor
    RosNode_RearCamera();
    // Inicializador de artefactos ROS
    void initROS();
    // Callbacks ROS
    void fcn_sub_ctrlRearCamera(CITIUS_Control_RearCamera::msg_ctrlRearCamera msg);
    bool fcv_serv_nodeStatus(CITIUS_Control_RearCamera::srv_nodeStatus::Request &rq, CITIUS_Control_RearCamera::srv_nodeStatus::Response &rsp);
    // Getter and Setter necesarios
    ros::Publisher getPubRearCameraInfo();
    short getFcNodeStatus();
    void setFcNodeStatus(short newFcNodeStatus);
    DrivingCameraManager *getDriverMng();
    
};

