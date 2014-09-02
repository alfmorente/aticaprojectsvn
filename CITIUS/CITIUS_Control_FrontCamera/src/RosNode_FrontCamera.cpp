#include "RosNode_FrontCamera.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_FrontCamera::RosNode_FrontCamera() {
    this->setFcNodeStatus(NODESTATUS_INIT);
    this->dFrontCamera = new AxisP3364LveDriver();
}

/*******************************************************************************
 * INICIALIZADOR DE ARTEFACTOS ROS
 ******************************************************************************/

void RosNode_FrontCamera::initROS() {
    ros::NodeHandle nh;
    this->pubFrontCameraInfo = nh.advertise<CITIUS_Control_FrontCamera::msg_frontCameraInfo>("frontCameraInfo",1000);
    this->subsCtrlFrontCamera = nh.subscribe("ctrlFrontCamera",1000,&RosNode_FrontCamera::fcn_sub_ctrlFrontCamera,this);
    this->servNodeStatus = nh.advertiseService("fcNodeStatus",&RosNode_FrontCamera::fcv_serv_nodeStatus,this);
}

// Control de la camara
void RosNode_FrontCamera::fcn_sub_ctrlFrontCamera(CITIUS_Control_FrontCamera::msg_ctrlFrontCamera msg){
    if(msg.isZoom) dFrontCamera->setZoom(msg.zoom);
    if(msg.isPan) dFrontCamera->setPan(msg.pan);
    if(msg.isZoom) dFrontCamera->setTilt(msg.tilt);
}
    
// Gestion del estado del nodo
bool RosNode_FrontCamera::fcv_serv_nodeStatus(CITIUS_Control_FrontCamera::srv_nodeStatus::Request &rq, CITIUS_Control_FrontCamera::srv_nodeStatus::Response &rsp){
    if(rq.status == NODESTATUS_OK){
        this->setFcNodeStatus(NODESTATUS_OK);
        rsp.confirmation = true;
    }else if(rq.status == NODESTATUS_OFF){
        this->setFcNodeStatus(NODESTATUS_OFF);
        rsp.confirmation = true;
    }else{
        rsp.confirmation = false;
    }
    return true;
}

/*******************************************************************************
 * GETTER AND SETTER NECESARIOS
 ******************************************************************************/

// Get del estado del nodo
short RosNode_FrontCamera::getFcNodeStatus(){
    return this->fcNodeStatus;
}

// Set del estado del nodo
void RosNode_FrontCamera::setFcNodeStatus(short newFcNodeStatus){
    this->fcNodeStatus = newFcNodeStatus;
}

// Obtener el publicador de informacion del vehiculo
ros::Publisher RosNode_FrontCamera::getPubFrontCameraInfo(){
    return this->pubFrontCameraInfo;
}

// Obtener el driver 
AxisP3364LveDriver *RosNode_FrontCamera::getDriverMng(){
    return this->dFrontCamera;
}