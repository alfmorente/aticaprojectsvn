#include "RosNode_FrontCamera.h"

/*******************************************************************************
 * CONSTRUCTOR DE LA CLASE
 ******************************************************************************/

RosNode_FrontCamera::RosNode_FrontCamera() {
    this->setFcNodeStatus(NODESTATUS_INIT);
    this->dFrontCamera = new DrivingCameraManager();
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

/*******************************************************************************
 * GETTER AND SETTER NECESARIOS
 ******************************************************************************/

// Control de la camara
void RosNode_FrontCamera::fcn_sub_ctrlFrontCamera(CITIUS_Control_FrontCamera::msg_ctrlFrontCamera msg){
    this->getDriverMng()->setParam(IDPARAM_PAN,msg.pan);
    this->getDriverMng()->setParam(IDPARAM_TILT,msg.tilt);
}
    
// Gestion del estado del nodo
bool RosNode_FrontCamera::fcv_serv_nodeStatus(CITIUS_Control_FrontCamera::srv_nodeStatus::Request &rq, CITIUS_Control_FrontCamera::srv_nodeStatus::Response &rsp){
    if(rq.status == NODESTATUS_OK){
        this->setFcNodeStatus(NODESTATUS_OK);
        rsp.confirmation = true;
    }else if(rq.status == NODESTATUS_OFF){
        this->getDriverMng()->disconnect();
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
DrivingCameraManager *RosNode_FrontCamera::getDriverMng(){
    return this->dFrontCamera;
}